// Host-side simulation of the self-learning TPI controller in
// boiler_temp_mqtt.ino. The tpi* functions are a line-by-line port of the
// firmware (same constants, same float/double split, same call order).
using System;
using System.Collections.Generic;

class Tpi {
    // ---- constants (copied from firmware) ----
    const ulong TPI_CYCLE_MS = 600000, TPI_MIN_ON_MS = 300000, TPI_MIN_OFF_MS = 180000;
    const float TPI_HORIZON_H = 1.5f, TPI_I_MAX = 0.25f;
    public static float TPI_TI_H = 2.0f;    // firmware TPI_TI_H
    const float TPI_OVERSHOOT = 0.4f, TPI_FULL_ON_ERR = 2.0f;
    const int TPI_MIN_SAMPLES = 100;
    const float HEAT_DEFAULT = 2.0f, LOSS_DEFAULT = 0.5f;
    const float HEAT_MIN = 0.3f, HEAT_MAX = 8.0f, LOSS_MAX = 4.0f;
    public static bool GAINFIX = true;
    public static bool LAG_KICK = true; public static float LAG_KICK_ERR = 0.3f;
    public static bool CAP7 = true;
    public static bool VARPWM = false;                 // variable-length cycles
    public static double MAX_STARTS = 3.0;             // per hour, for choosing ON length
    const double MAX_PERIOD_MIN = 40.0, FULL_DUTY = 0.85;
    ulong cyclePeriodMs = TPI_CYCLE_MS;
    public double onSum = 0; public int onPulses = 0;
    static float LOSS_MIN { get { return GAINFIX ? 0.2f : 0.0f; } }
    const float RLS_LAMBDA = 0.97f, RLS_MAX_SLOPE = 6.0f;
    public static bool FIXES = true;
    public static bool ROUND_NEAREST = true;
    const float TPI_EXCITE = 0.25f, TPI_LOSS_GAIN = 0.1f, TPI_CARRY_MAX = 0.5f;
    float dutyCarry = 0, uAvg = -1;
    // Lag-aware learning: rHat follows the boiler through a first-order
    // filter (the radiator), and the cycle's mean rHat replaces uEff.
    public static double LAG_TAU_MIN = 12;    // firmware TPI_RAD_TAU_MIN; 0 = old 0.6/0.4 duty weighting
    public static bool LAGPRED = false;       // subtract heat still stored in the radiator
    public static double PRED_GAIN = 1.0;
    double rHat = 0, rSum = 0; int rN = 0; ulong lastMs = 0; bool lastOn = false;
    void LagUpdate(ulong now) {
        if (LAG_TAU_MIN > 0 && now > lastMs) { double dt = (now - lastMs) / 60000.0; rHat += ((lastOn ? 1.0 : 0.0) - rHat) * (1 - Math.Exp(-dt / LAG_TAU_MIN)); }
        lastMs = now;
    }

    // ---- state ----
    public float temperature, target = 22.0f;
    public float heatRate = HEAT_DEFAULT, lossRate = LOSS_DEFAULT, integ = 0;
    public int learnN = 0;
    float[,] rlsP = { { 4.0f, 0 }, { 0, 1.0f } };   // fresh model (learnN < 10)
    public bool cycleActive = false;
    bool skipLearnNext = true, cycleLearnable = false;
    ulong cycleStartMs = 0, cycleOnMs = 0;
    public float cycleDuty = 0, prevActualDuty = 0;
    double sX, sY, sXX, sXY; int sN;
    public ulong millis;
    public int rejected = 0, cutCount = 0, gainSkips = 0, lagKicks = 0;
    public static int GAINRULE = 1; public static bool SKIPCUT = false; public static float ECLIP = 0; bool cycleCut = false;
    public static float WLO = 1, WHI = 1; float startErr = 0; float curW = 1;

    static float Clamp(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

    public float tpiKp() { return 1.0f / (heatRate * TPI_HORIZON_H); }

    float tpiComputeDuty() {
        float err = target - temperature;
        if (err >= TPI_FULL_ON_ERR) return 1.0f;
        if (err <= -TPI_OVERSHOOT)  return 0.0f;
        if (LAGPRED && LAG_TAU_MIN > 0) err -= (float)(PRED_GAIN * heatRate * rHat * LAG_TAU_MIN / 60.0);
        float b = lossRate / heatRate;
        float duty = b + tpiKp() * err + integ;
        return Clamp(duty, 0.0f, 1.0f);
    }

    public void tpiAbort() { LagUpdate(millis); lastOn = false; cycleActive = false; skipLearnNext = true; dutyCarry = 0; }

    void tpiLearnLossOnly(float u, float slope) {
        float e = slope - (heatRate * u - lossRate); if (ECLIP > 0) e = Clamp(e, -ECLIP, ECLIP);
        lossRate = Clamp(lossRate - Math.Min(1f, TPI_LOSS_GAIN * curW) * e, LOSS_MIN, LOSS_MAX);
        learnN++;
    }

    void tpiLearn(float u, float slope) {
        float[] phi = { u, -1.0f };
        float[] Pphi = { rlsP[0,0]*phi[0] + rlsP[0,1]*phi[1], rlsP[1,0]*phi[0] + rlsP[1,1]*phi[1] };
        float denom = RLS_LAMBDA / curW + phi[0]*Pphi[0] + phi[1]*Pphi[1];
        float[] k = { Pphi[0]/denom, Pphi[1]/denom };
        float predicted = heatRate * u - lossRate;
        float e = slope - predicted; if (ECLIP > 0) e = Clamp(e, -ECLIP, ECLIP);
        heatRate = Clamp(heatRate + k[0]*e, HEAT_MIN, HEAT_MAX);
        lossRate = Clamp(lossRate + k[1]*e, LOSS_MIN, LOSS_MAX);
        float[,] newP = new float[2,2];
        for (int i = 0; i < 2; i++) for (int j = 0; j < 2; j++)
            newP[i,j] = (rlsP[i,j] - k[i]*Pphi[j]) / RLS_LAMBDA;
        float off = 0.5f * (newP[0,1] + newP[1,0]);
        newP[0,1] = newP[1,0] = off;
        float tr = newP[0,0] + newP[1,1];
        if (tr > 10.0f) { float s = 10.0f/tr; for (int i=0;i<2;i++) for (int j=0;j<2;j++) newP[i,j] *= s; }
        rlsP = newP;
        learnN++;
    }

    void tpiEndCycle() {
        ulong per = Math.Max(1UL, Math.Min(cyclePeriodMs, millis - cycleStartMs));
        float actualDuty = (float)Math.Min(cycleOnMs, per) / (float)per;
        if (cycleLearnable && sN >= TPI_MIN_SAMPLES) {
            double den = sN * sXX - sX * sX;
            if (den > 1e-9) {
                float slope = (float)((sN * sXY - sX * sY) / den);
                if (Math.Abs(slope) <= RLS_MAX_SLOPE) {
                    { float a = Clamp((startErr - 0.2f) / 0.8f, 0f, 1f); curW = WLO + (WHI - WLO) * a; }
                    float uEff = (LAG_TAU_MIN > 0 && rN > 0) ? (float)(rSum / rN) : 0.6f * actualDuty + 0.4f * prevActualDuty;
                    if (!FIXES) tpiLearn(uEff, slope);
                    else {
                        bool excited = (uAvg < 0) || Math.Abs(uEff - uAvg) >= TPI_EXCITE;
                        uAvg = (uAvg < 0) ? uEff : 0.8f * uAvg + 0.2f * uEff;
                        if (SKIPCUT && cycleCut) { gainSkips++; }
                        else if (GAINFIX && slope > 0.3f + heatRate * uEff && (GAINRULE == 1 ? actualDuty <= 0.0f : (LAG_TAU_MIN > 0 ? uEff < 0.1f : uEff <= 0.0f))) { gainSkips++; }
                        else if (excited) tpiLearn(uEff, slope); else tpiLearnLossOnly(uEff, slope);
                    }
                } else rejected++;
            }
        }
        prevActualDuty = actualDuty;
        cycleActive = false;
    }

    void tpiStartCycle(ulong now) {
        startErr = Math.Abs(target - temperature);
        float err = target - temperature;
        float duty = tpiComputeDuty();
        bool satHi = (duty >= 1.0f && err > 0), satLo = (duty <= 0.0f && err < 0);
        if (!skipLearnNext && !satHi && !satLo && Math.Abs(err) < TPI_FULL_ON_ERR) {
            float cycleH = cyclePeriodMs / 3600000.0f;
            integ = Clamp(integ + (tpiKp() / TPI_TI_H) * err * cycleH, -TPI_I_MAX, TPI_I_MAX);
            duty = tpiComputeDuty();
        }
        ulong onMs;
        cyclePeriodMs = TPI_CYCLE_MS;
        if (VARPWM) {
            // Choose the ON length from the demand: short burns when little heat
            // is needed, long ones when a lot is, at most MAX_STARTS per hour;
            // the OFF time follows so the average is exactly the demand.
            if (duty <= 0.02f) { onMs = 0; cyclePeriodMs = TPI_CYCLE_MS; }
            else if (duty >= FULL_DUTY) { onMs = cyclePeriodMs = TPI_CYCLE_MS; }
            else {
                double onMin  = Math.Max(TPI_MIN_ON_MS / 60000.0, duty * 60.0 / MAX_STARTS);
                double offMin = onMin * (1 - duty) / duty;
                if (offMin < TPI_MIN_OFF_MS / 60000.0) { onMs = cyclePeriodMs = TPI_CYCLE_MS; }
                else {
                    if (onMin + offMin > MAX_PERIOD_MIN) offMin = MAX_PERIOD_MIN - onMin;
                    onMs = (ulong)(onMin * 60000); cyclePeriodMs = (ulong)((onMin + offMin) * 60000);
                }
            }
            if (onMs > 0) { onSum += onMs / 60000.0; onPulses++; }
        } else if (!FIXES) {
            onMs = (ulong)(duty * TPI_CYCLE_MS);
            if (onMs < TPI_MIN_ON_MS) onMs = 0;
            else if (TPI_CYCLE_MS - onMs < TPI_MIN_OFF_MS) onMs = TPI_CYCLE_MS;
        } else {
            if (duty <= 0.0f) dutyCarry = 0.0f;
            float want = Clamp(duty + dutyCarry, 0.0f, 1.0f);
            onMs = (ulong)(want * TPI_CYCLE_MS);
            if (onMs < TPI_MIN_ON_MS) onMs = (ROUND_NEAREST && onMs * 2 >= TPI_MIN_ON_MS) ? TPI_MIN_ON_MS : 0;
            if (LAG_KICK && onMs == 0 && err >= LAG_KICK_ERR && duty > 0) { onMs = TPI_MIN_ON_MS; lagKicks++; }
            else if (TPI_CYCLE_MS - onMs < TPI_MIN_OFF_MS) onMs = (!CAP7 || duty >= 0.5f || onMs * 2 >= 2 * TPI_CYCLE_MS - TPI_MIN_OFF_MS) ? TPI_CYCLE_MS : TPI_CYCLE_MS - TPI_MIN_OFF_MS;
            dutyCarry = Clamp(duty + dutyCarry - (float)onMs / TPI_CYCLE_MS, -TPI_CARRY_MAX, TPI_CARRY_MAX);
        }
        cycleActive = true;
        cycleLearnable = !skipLearnNext;
        skipLearnNext = false;
        cycleStartMs = now; cycleOnMs = onMs; cycleDuty = duty; cycleCut = false;
        sX = sY = sXX = sXY = 0; sN = 0; rSum = 0; rN = 0;
    }

    public bool tpiControl() {
        ulong now = millis;
        if (cycleActive && now - cycleStartMs >= cyclePeriodMs) tpiEndCycle();
        // Variable cycles can have long OFF parts: re-plan early if the room
        // has clearly fallen behind (after the minimum OFF time).
        if (VARPWM && cycleActive && now - cycleStartMs >= cycleOnMs + TPI_MIN_OFF_MS
            && (target - temperature) >= LAG_KICK_ERR) tpiEndCycle();
        LagUpdate(now);
        if (!cycleActive) tpiStartCycle(now);
        rSum += rHat; rN++;
        double x = (now - cycleStartMs) / 3600000.0;
        sX += x; sY += temperature; sXX += x*x; sXY += x*temperature; sN++;
        bool inOn = (now - cycleStartMs) < cycleOnMs;
        if (inOn && temperature >= target + TPI_OVERSHOOT && (now - cycleStartMs) >= TPI_MIN_ON_MS) {
            cycleOnMs = now - cycleStartMs; inOn = false; cutCount++; cycleCut = true;
        }
        lastOn = inOn;
        return inOn;
    }
}

// ---- Port of doorDetect() ----
class Door {
    const int BUF = 100, SLOPE_N = 40, MIN_N = 20;
    const float SLOPE_OPEN = -0.25f, DROP_OPEN = 0.6f, SLOPE_STABLE = -0.05f;
    const ulong CLOSE_DELAY = 180000;
    float[] t = new float[BUF]; ulong[] ms = new ulong[BUF];
    int idx = 0, filled = 0; ulong closeAt = 0;
    public bool open = false; public float slope;
    int Ago(int i) { return (idx - 1 - i + 2 * BUF) % BUF; }
    public void Add(float temp, ulong now) {
        t[idx] = temp; ms[idx] = now; idx = (idx + 1) % BUF; if (filled < BUF) filled++;
        if (filled < MIN_N) return;
        int n = Math.Min(filled, SLOPE_N);
        ulong ms0 = ms[Ago(n - 1)];
        double sx = 0, sy = 0, sxx = 0, sxy = 0;
        for (int i = 0; i < n; i++) { int k = Ago(i); double x = (ms[k] - ms0) / 60000.0; sx += x; sy += t[k]; sxx += x * x; sxy += x * t[k]; }
        double den = n * sxx - sx * sx; if (den < 1e-9) return;
        slope = (float)((n * sxy - sx * sy) / den);
        float peak = -100; for (int i = 0; i < filled; i++) peak = Math.Max(peak, t[Ago(i)]);
        float drop = peak - t[Ago(0)];
        if (!open) {
            if (slope <= SLOPE_OPEN || drop >= DROP_OPEN) { open = true; closeAt = 0; }
        } else {
            if (slope > SLOPE_STABLE) {
                if (closeAt == 0) closeAt = now;
                else if (now - closeAt >= CLOSE_DELAY) { open = false; closeAt = 0; filled = 0; }
            } else closeAt = 0;
        }
    }
}

// ---- Room + radiator plant ----
// dT/dt [°C/h] = a·r − b·(T − Tout),  dr/dt = (boiler − r)/tauRad
class Room {
    public double T, r = 0, w = 0, a, b, tauRadH, tauWaterH = 0;
    public double windowLoss = 0;               // extra °C/h while a window/door is open
    public double solar = 0;                    // extra °C/h from sun / other heat sources
    public double sensor;                       // DHT22 reading lags the air (τ ≈ 1.5 min)
    public Room(double T0, double a_, double b_, double tauMin) { T = T0; sensor = T0; a = a_; b = b_; tauRadH = tauMin / 60.0; }
    public void Step(bool boiler, double tout, double dtH) {
        double drive = boiler ? 1.0 : 0.0;
        if (tauWaterH > 0) { w += (drive - w) * dtH / tauWaterH; drive = w; }
        r += (drive - r) * dtH / tauRadH;
        T += (a * r - b * (T - tout) - windowLoss + solar) * dtH;
        sensor += (T - sensor) * dtH / (1.5 / 60.0);
    }
}

class Program {
    static Random rng = new Random(42);
    static Func<double,double> SolarProfile = h => 0;
    static double Gauss() { double u1 = 1.0 - rng.NextDouble(), u2 = rng.NextDouble(); return Math.Sqrt(-2*Math.Log(u1))*Math.Cos(2*Math.PI*u2); }

    class Seg { public double fromH, toH; public float target; }
    class Win { public double atH, minutes, lossPerH; public string label; }
    static Win[] AiringDays(int days) {
        var l = new List<Win>();
        for (int d = 0; d < days; d++) {
            l.Add(new Win { atH = d * 24 + 15.5, minutes = 22, lossPerH = 6, label = "Day " + d + " airing" });
            l.Add(new Win { atH = d * 24 + 15.5 + 22 / 60.0, minutes = 40, lossPerH = 1.5, label = "Day " + d + " after-airing" });
        }
        return l.ToArray();
    }

    static void Run(string name, Room room, double days, Seg[] targets, Func<double,double> tout, string note) {
        Run(name, room, days, targets, tout, note, new Win[0], true);
    }

    static void Run(string name, Room room, double days, Seg[] targets, Func<double,double> tout, string note, Win[] wins, bool table) {
        var door = new Door();
        bool prevDoor = false; double doorOpenedAtH = 0;
        var doorEvents = new List<string>();
        var eveMin = new Dictionary<int,double>(); var eveLow = new Dictionary<int,double>();
        Console.WriteLine();
        Console.WriteLine("=== " + name + " ===");
        Console.WriteLine(note);
        var tpi = new Tpi();
        const double dtS = 3.0;
        double dtH = dtS / 3600.0;
        long steps = (long)(days * 86400 / dtS);
        bool boiler = false, prevBoiler = false;
        double meanErr = 0; long atTarget = 0, offBy2 = 0;
        int starts = 0; double onMinTotal = 0, aboveMin = 0, settledSq = 0; long settledN = 0;
        // per-6h buckets
        double bSum = 0, bSq = 0, bMin = 99, bMax = -99, bOn = 0; int bN = 0, bStarts = 0;
        float curTarget = targets[0].target;
        Console.WriteLine("  hours | target | T mean   min    max  | RMS err | boiler% | starts | learned H    L   (true at target: H~{0:F2} L~{1:F2}) | I",
            room.a, 0);
        for (long i = 0; i <= steps; i++) {
            double hours = i * dtH;
            foreach (var s in targets) if (hours >= s.fromH && hours < s.toH && s.target != curTarget) {
                curTarget = s.target; tpi.target = curTarget; tpi.tpiAbort();
            }
            tpi.target = curTarget;
            room.solar = SolarProfile(hours);
            // Window events
            room.windowLoss = 0; Win active = null;
            foreach (var w in wins) if (hours >= w.atH && hours < w.atH + w.minutes / 60.0) { room.windowLoss = w.lossPerH; active = w; }
            // DHT22: lagging sensor + noise + 0.1 °C resolution
            float meas = (float)(Math.Round((room.sensor + 0.05 * Gauss()) * 10) / 10.0);
            tpi.temperature = meas;
            tpi.millis = (ulong)(i * dtS * 1000);
            door.Add(meas, tpi.millis);
            if (door.open != prevDoor) {
                Win near = null;   // a window that is open now or closed less than 20 min ago
                foreach (var w in wins) if (hours >= w.atH && hours < w.atH + (w.minutes + 20) / 60.0) near = w;
                if (door.open) {
                    doorOpenedAtH = hours;
                    doorEvents.Add(near == null
                        ? string.Format("    FALSE ALARM at {0:F2} h (slope {1:F2} °C/min)", hours, door.slope)
                        : string.Format("    {0}: detected {1:F1} min after opening", near.label, (hours - near.atH) * 60));
                } else {
                    doorEvents.Add(string.Format("      closed again after {0:F1} min{1}", (hours - doorOpenedAtH) * 60,
                        near != null ? string.Format(" ({0:F1} min after the window closed)", (hours - near.atH) * 60 - near.minutes) : ""));
                }
                prevDoor = door.open;
            }
            if (door.open) { tpi.tpiAbort(); boiler = false; }
            else boiler = tpi.tpiControl();
            if (boiler && !prevBoiler) { starts++; bStarts++; }
            if (boiler) onMinTotal += dtS / 60.0;
            if (hours > 12) { if (room.T > curTarget + 0.2) aboveMin += dtS / 60.0; meanErr += meas - curTarget; if (Math.Abs(meas - curTarget) < 0.05) atTarget++; if (Math.Abs(meas - curTarget) > 0.15) offBy2++; settledSq += (room.T - curTarget) * (room.T - curTarget); settledN++; }
            prevBoiler = boiler;
            room.Step(boiler, tout(hours), dtH);
            { int day = (int)(hours / 24); double hd = hours % 24; if (hd >= 16 && hd < 19) { if (!eveMin.ContainsKey(day) || room.T < eveMin[day]) eveMin[day] = room.T; if (room.T < curTarget - 0.3) eveLow[day] = (eveLow.ContainsKey(day) ? eveLow[day] : 0) + dtH * 60; } }

            double err = room.T - curTarget;
            bSum += room.T; bSq += err * err; bN++; if (room.T < bMin) bMin = room.T; if (room.T > bMax) bMax = room.T; if (boiler) bOn++;
            if (table && bN > 0 && Math.Abs((hours % 6.0)) < dtH / 2 && i > 0) {
                double trueL = room.b * (curTarget - tout(hours));
                Console.WriteLine("  {0,5:F0} | {1,6:F1} | {2,6:F2} {3,6:F2} {4,6:F2} | {5,7:F2} | {6,6:F0}% | {7,6} | {8,10:F2} {9,5:F2}   (true H={10:F2} L={11:F2})                | {12,6:F3}",
                    hours, curTarget, bSum / bN, bMin, bMax, Math.Sqrt(bSq / bN), 100 * bOn / bN, bStarts,
                    tpi.heatRate, tpi.lossRate, room.a, trueL, tpi.integ);
                bSum = bSq = bOn = 0; bN = 0; bStarts = 0; bMin = 99; bMax = -99;
            }
        }
        Console.WriteLine("  sensor after 12 h: mean offset {0:+0.000;-0.000} C, reads exactly target {1:F0}%, off by >=0.2 {2:F0}%", meanErr / Math.Max(1, settledN), 100.0 * atTarget / Math.Max(1, settledN), 100.0 * offBy2 / Math.Max(1, settledN));
        Console.WriteLine("  avg ON pulse {0:F1} min, after 12 h: RMS {1:F3} °C, {2:F0} min/day above target+0.2", starts > 0 ? onMinTotal / starts : 0, Math.Sqrt(settledSq / Math.Max(1, settledN)), aboveMin / Math.Max(0.01, (days * 24 - 12) / 24));
        Console.WriteLine("  final learned H {0:F2} L {1:F2} (L/H {2:F2}; true H {3:F2}, L at target {4:F2}, L/H {5:F2}), I {6:F3}",
            tpi.heatRate, tpi.lossRate, tpi.lossRate / tpi.heatRate, room.a, room.b * (curTarget - tout(days * 24)), room.b * (curTarget - tout(days * 24)) / room.a, tpi.integ);
        Console.WriteLine("  gain-skipped learning cycles: {0}, undershoot kicks: {1}", tpi.gainSkips, tpi.lagKicks);
        Console.WriteLine("  total starts: {0} ({1:F1}/h), learned cycles: {2}, rejected slopes: {3}, overshoot cuts: {4}",
            starts, starts / (days * 24), tpi.learnN, tpi.rejected, tpi.cutCount);
        foreach (var kv in eveMin) Console.WriteLine("  day {0} 16-19h: min {1:F2} °C, {2:F0} min more than 0.3 below target", kv.Key, kv.Value, eveLow.ContainsKey(kv.Key) ? eveLow[kv.Key] : 0);
        Console.WriteLine("  door detector: " + (doorEvents.Count == 0 ? "no alarms" : ""));
        foreach (var e in doorEvents) Console.WriteLine(e);
    }

    static void Main(string[] args) {
        Tpi.FIXES = !(args.Length > 0 && args[0] == "old");
        Tpi.ROUND_NEAREST = (args.Length > 0 && args[0] == "nearest");   // firmware: skip + kick on target raise
        Console.WriteLine(Tpi.ROUND_NEAREST ? "##### ROUND TO NEAREST #####" : "##### SKIP SHORT PULSES #####");
        rng = new Random(42);
        Console.WriteLine(Tpi.FIXES ? "##### WITH FIXES #####" : "##### ORIGINAL #####");
        Tpi.GAINFIX = !(args.Length > 1 && args[1] == "nogainfix");
        Tpi.LAG_KICK = !(args.Length > 2 && args[2] == "nokick");
        Tpi.CAP7 = !(args.Length > 3 && args[3] == "up");
        Tpi.VARPWM = (args.Length > 3 && args[3] == "var");
        if (args.Length > 4) Tpi.MAX_STARTS = double.Parse(args[4], System.Globalization.CultureInfo.InvariantCulture);
        if (Tpi.VARPWM) Console.WriteLine("##### VARIABLE CYCLES, max starts/h " + Tpi.MAX_STARTS + " #####");
        Console.WriteLine(Tpi.CAP7 ? "##### ROUND DOWN TO 7 MIN #####" : "##### ROUND UP TO 10 MIN #####");
        Console.WriteLine(Tpi.LAG_KICK ? "##### UNDERSHOOT KICK >= 0.3 #####" : "##### NO UNDERSHOOT KICK #####");
        Console.WriteLine(Tpi.GAINFIX ? "##### GAIN FIX (skip u=0 & rising, L >= 0.2) #####" : "##### NO GAIN FIX #####");
        var inv = System.Globalization.CultureInfo.InvariantCulture;
        string ev = Environment.GetEnvironmentVariable("LAGTAU"); if (!string.IsNullOrEmpty(ev)) Tpi.LAG_TAU_MIN = double.Parse(ev, inv);
        Tpi.LAGPRED = Environment.GetEnvironmentVariable("LAGPRED") == "1";
        ev = Environment.GetEnvironmentVariable("PREDGAIN"); if (!string.IsNullOrEmpty(ev)) Tpi.PRED_GAIN = double.Parse(ev, inv);
        Console.WriteLine("##### LAG TAU " + Tpi.LAG_TAU_MIN + " min, predict " + Tpi.LAGPRED + " x" + Tpi.PRED_GAIN + " #####");
        ev = Environment.GetEnvironmentVariable("TI"); if (!string.IsNullOrEmpty(ev)) Tpi.TPI_TI_H = float.Parse(ev, inv);
        ev = Environment.GetEnvironmentVariable("WLO"); if (!string.IsNullOrEmpty(ev)) Tpi.WLO = float.Parse(ev, inv);
        ev = Environment.GetEnvironmentVariable("WHI"); if (!string.IsNullOrEmpty(ev)) Tpi.WHI = float.Parse(ev, inv);
        ev = Environment.GetEnvironmentVariable("GAINRULE"); if (!string.IsNullOrEmpty(ev)) Tpi.GAINRULE = int.Parse(ev);
        Tpi.SKIPCUT = Environment.GetEnvironmentVariable("SKIPCUT") == "1";
        ev = Environment.GetEnvironmentVariable("ECLIP"); if (!string.IsNullOrEmpty(ev)) Tpi.ECLIP = float.Parse(ev, inv);
        ev = Environment.GetEnvironmentVariable("SUNAM"); if (!string.IsNullOrEmpty(ev)) { double am = double.Parse(ev, inv); SolarProfile = h => { double d = h % 24; return (d >= 8 && d <= 12) ? am * Math.Sin(Math.PI * (d - 8) / 4) : 0; }; }
        if (args.Length > 1 && args[1] == "real") {
            // G/H) Like the user's flat: two-stage lag (boiler water, then radiator mass),
            //      needs ~40 % at 22 °C, outdoor 10±4 °C, daily airing.
            var gs = new[] { new { a = 3.0, b = 0.10, tw = 8.0, tr = 15.0, n = "G) Real-like flat: +3 C/h, loss 1.2 C/h at 22, lag 8+15 min" },
                             new { a = 3.0, b = 0.10, tw = 12.0, tr = 25.0, n = "H) Slow radiators: +3 C/h, lag 12+25 min" } };
            foreach (var g in gs) {
                var room = new Room(21, g.a, g.b, g.tr); room.tauWaterH = g.tw / 60.0;
                Run(g.n, room, 4, new[] { new Seg { fromH = 0, toH = 1e9, target = 22f } },
                    h => 10 + 4 * Math.Sin(2 * Math.PI * (h - 9) / 24), "", Environment.GetEnvironmentVariable("AIR") == "0" ? new Win[0] : AiringDays(4), Environment.GetEnvironmentVariable("TABLE") == "1");
            }
            return;
        }
        if (args.Length > 1 && args[1] != "all") {
            // F) Sunny afternoons: up to +4 °C/h of free heat 10:00–15:00 local
            //    (hours here start at midnight). Like the user's 7 Nov report.
            SolarProfile = h => { double d = h % 24; return (d >= 10 && d <= 15) ? 1.5 * Math.Sin(Math.PI * (d - 10) / 5) : 0; };
            Run("F) Sunny afternoons (+1.5 °C/h free heat 10-15h), target 22, outdoor 5±5 °C",
                new Room(21, 3.1, 0.0375, 12), 4,
                new[] { new Seg { fromH = 0, toH = 1e9, target = 22f } },
                h => 5 + 5 * Math.Sin(2 * Math.PI * (h - 9) / 24), "Daily 15:30 airing 22 min, then the cooled room keeps losing heat for 40 min",
                AiringDays(4), true);
            return;
        }
        // Typical: full power +3.1 °C/h, loses 0.6 °C/h at 21 °C with 5 °C outside, radiator lag 12 min
        Run("A) Typical house, cold start 17 °C -> target 21, outdoor 5±5 °C daily",
            new Room(17, 3.1, 0.0375, 12), 3,
            new[] { new Seg { fromH = 0, toH = 1e9, target = 21f } },
            h => 5 + 5 * Math.Sin(2 * Math.PI * (h - 9) / 24), "");

        Run("B) Target changes: 21 -> 23 at 30 h -> 19 at 42 h",
            new Room(20, 3.1, 0.0375, 12), 2.5,
            new[] { new Seg { fromH = 0, toH = 30, target = 21f }, new Seg { fromH = 30, toH = 42, target = 23f }, new Seg { fromH = 42, toH = 1e9, target = 19f } },
            h => 5, "");

        Run("C) Weak heating (+1.2 °C/h), strong loss, outdoor 0 °C",
            new Room(18, 1.2, 0.04, 20), 6,
            new[] { new Seg { fromH = 0, toH = 1e9, target = 21f } },
            h => 0, "Expect: high duty, slower rise, learner should find H~1.2");

        Run("D) Powerful heating (+8 °C/h), well insulated, outdoor 10 °C",
            new Room(19, 8.0, 0.02, 8), 3,
            new[] { new Seg { fromH = 0, toH = 1e9, target = 21f } },
            h => 10, "Expect: short pulses, little overshoot, learner should raise H");

        // Window/door openings in the typical house, outdoor 5 °C.
        // lossPerH = extra cooling of the room air while open (°C/h).
        var wins = new[] {
            new Win { atH = 20, minutes = 10, lossPerH = 18, label = "Window wide open 10 min (-3 °C/10min)" },
            new Win { atH = 26, minutes = 10, lossPerH = 9,  label = "Window open 10 min (-1.5 °C/10min)" },
            new Win { atH = 32, minutes = 10, lossPerH = 5,  label = "Window ajar 10 min (-0.8 °C/10min)" },
            new Win { atH = 38, minutes = 3,  lossPerH = 18, label = "Front door 3 min (-0.9 °C/3min)" },
            new Win { atH = 44, minutes = 30, lossPerH = 6,  label = "Airing 30 min (-3 °C/30min)" },
            new Win { atH = 50, minutes = 10, lossPerH = 3,  label = "Window barely open 10 min (-0.5 °C/10min)" },
        };
        Run("E) Window / door openings (typical house)", new Room(21, 3.1, 0.0375, 12), 2.5,
            new[] { new Seg { fromH = 0, toH = 1e9, target = 21f } }, h => 5, "", wins, false);
    }
}
