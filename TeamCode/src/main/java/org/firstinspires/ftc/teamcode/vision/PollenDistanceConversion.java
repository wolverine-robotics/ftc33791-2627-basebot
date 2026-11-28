package org.firstinspires.ftc.teamcode.vision;

import java.util.Locale;

/**
 * PollenDistanceConversion - turns Limelight tx/ty angles into the floor position of a pollen.
 *
 * PURE JAVA (Java 8). No FTC imports, so it runs on a PC and on the Control Hub unchanged.
 * Allocation-free in normal use: make one instance, reuse one Buffer.
 *
 * OUTPUT FRAME (inches):  X = right (+) / left (-),  Y = forward (+).
 * With the calibrated defaults below, Y is measured from the line your ruler's Y = 0 was on
 * (about 6.8 in ahead of the camera lens). To measure from robot center instead, set
 * config().cameraOffsetForwardIn to the camera's forward position relative to robot center.
 *
 * HOW: tx/ty define a ray from the camera. It is tilted by the camera's pitch/yaw and intersected
 * with the horizontal plane at the height of the ball's center (ball radius). That point is X, Y.
 *
 * Normally you don't use this class directly - PollenDetector does. Use it directly only if you
 * already have tx/ty from somewhere else.
 */
public class PollenDistanceConversion {

    // =====================================================================
    //  CONFIG  (defaults = ruler-calibrated for the current rig + pipeline)
    // =====================================================================
    public static class Config {
        /** Camera lens center height above the floor, inches (measured). */
        public double cameraHeightIn = 7.0;
        /**
         * Effective downward tilt, degrees. CALIBRATED = 9.75 (the rig's nominal 70 deg = 20 deg down was
         * ~10 deg off; part of that is probably the pipeline crosshair at y = 0.25). Valid only for the
         * current pipeline/crosshair/mount - if any of those change, refit.
         */
        public double cameraPitchDownDeg = 9.75;
        /** Camera yaw, degrees (+ = turned right). Calibrated. */
        public double cameraYawDeg = -1.06;
        /** Camera position relative to the output origin: right (+), inches. */
        public double cameraOffsetRightIn = 0.0;
        /** Camera position relative to the output origin: forward (+), inches. Calibrated (see class note). */
        public double cameraOffsetForwardIn = -6.76;
        /** Added to reported tx / ty (only needed if you later change the crosshair instead of refitting). */
        public double txOffsetDeg = 0.0;
        public double tyOffsetDeg = 0.0;
        /** Pollen diameter, inches. */
        public double ballDiameterIn = 2.8;

        public Config copy() {
            Config c = new Config();
            c.cameraHeightIn = cameraHeightIn;
            c.cameraPitchDownDeg = cameraPitchDownDeg;
            c.cameraYawDeg = cameraYawDeg;
            c.cameraOffsetRightIn = cameraOffsetRightIn;
            c.cameraOffsetForwardIn = cameraOffsetForwardIn;
            c.txOffsetDeg = txOffsetDeg;
            c.tyOffsetDeg = tyOffsetDeg;
            c.ballDiameterIn = ballDiameterIn;
            return c;
        }

        /** Lines to paste back into this file's defaults after a refit. */
        public String toJavaSnippet() {
            return String.format(Locale.US,
                    "cameraHeightIn = %.3f;%ncameraPitchDownDeg = %.3f;%ncameraYawDeg = %.3f;%n"
                            + "cameraOffsetRightIn = %.3f;%ncameraOffsetForwardIn = %.3f;%n"
                            + "txOffsetDeg = %.3f;%ntyOffsetDeg = %.3f;%nballDiameterIn = %.3f;",
                    cameraHeightIn, cameraPitchDownDeg, cameraYawDeg, cameraOffsetRightIn,
                    cameraOffsetForwardIn, txOffsetDeg, tyOffsetDeg, ballDiameterIn);
        }
    }

    // =====================================================================
    //  BUFFER - fixed-size, nearest-first list of results. Allocate once, clear() each look.
    // =====================================================================
    public static class Buffer {
        public final double[] x;        // inches, right +
        public final double[] y;        // inches, forward +
        public final double[] rangeIn;  // straight-line floor distance from the output origin
        public int count = 0;

        public Buffer(int capacity) {
            x = new double[capacity];
            y = new double[capacity];
            rangeIn = new double[capacity];
        }

        public void clear() {
            count = 0;
        }

        /** Insert keeping nearest first. If full, a ball farther than the last one is dropped. */
        public void add(double xIn, double yIn) {
            double r = Math.hypot(xIn, yIn);
            int cap = x.length;
            int i;
            if (count < cap) {
                i = count++;
            } else {
                if (r >= rangeIn[cap - 1]) return;
                i = cap - 1;
            }
            while (i > 0 && rangeIn[i - 1] > r) {
                x[i] = x[i - 1];
                y[i] = y[i - 1];
                rangeIn[i] = rangeIn[i - 1];
                i--;
            }
            x[i] = xIn;
            y[i] = yIn;
            rangeIn[i] = r;
        }
    }

    // =====================================================================
    //  INSTANCE
    // =====================================================================
    private final Config cfg;
    private final double[] scratch = new double[2];

    public PollenDistanceConversion() {
        this(new Config());
    }

    public PollenDistanceConversion(Config cfg) {
        this.cfg = cfg;
    }

    public Config config() {
        return cfg;
    }

    /** Convert one blob (tx/ty degrees) and add it to the buffer. False if the ray never hits the floor. */
    public boolean addTo(Buffer out, double txDeg, double tyDeg) {
        if (!toFloor(cfg, txDeg, tyDeg, scratch)) return false;
        out.add(scratch[0], scratch[1]);
        return true;
    }

    // =====================================================================
    //  CORE MATH
    // =====================================================================
    /** Writes X,Y (inches) into out[0], out[1]. Returns false if the ray points at/above the horizon. */
    public static boolean toFloor(Config c, double txDeg, double tyDeg, double[] out) {
        double th = Math.toRadians(c.cameraPitchDownDeg);
        double s = Math.sin(th);
        double co = Math.cos(th);
        double x = Math.tan(Math.toRadians(txDeg + c.txOffsetDeg));
        double y = Math.tan(Math.toRadians(tyDeg + c.tyOffsetDeg));
        double fwdRate = co + s * y;     // forward travel per unit of camera depth
        double upRate = -s + co * y;     // vertical travel per unit of camera depth
        double dz = c.ballDiameterIn / 2.0 - c.cameraHeightIn; // ball center vs camera (negative = below)
        if (dz >= -1e-9 || upRate >= -1e-9) return false;
        double t = dz / upRate;
        double xc = t * x;
        double yc = t * fwdRate;
        double psi = Math.toRadians(c.cameraYawDeg);
        out[0] = xc * Math.cos(psi) + yc * Math.sin(psi) + c.cameraOffsetRightIn;
        out[1] = -xc * Math.sin(psi) + yc * Math.cos(psi) + c.cameraOffsetForwardIn;
        return true;
    }

    // =====================================================================
    //  CALIBRATION (offline / PC use; never called by the robot at runtime)
    //  pts rows: {txDeg, tyDeg, measuredXIn, measuredYIn}
    // =====================================================================
    public static double rmsErrorIn(Config c, double[][] pts) {
        double[] o = new double[2];
        double sum = 0;
        for (double[] p : pts) {
            if (toFloor(c, p[0], p[1], o)) {
                sum += (o[0] - p[2]) * (o[0] - p[2]) + (o[1] - p[3]) * (o[1] - p[3]);
            } else {
                sum += 1e4;
            }
        }
        return Math.sqrt(sum / pts.length);
    }

    /** Fit the chosen parameters (pitch, height, yaw, forward offset) to ruler-measured points. */
    public static Config fit(Config start, double[][] pts,
                             boolean pitch, boolean height, boolean yaw, boolean fwd) {
        final boolean[] on = {pitch, height, yaw, fwd};
        double[] p = {start.cameraPitchDownDeg, start.cameraHeightIn, start.cameraYawDeg,
                start.cameraOffsetForwardIn};
        double[] step = {2.0, 1.0, 1.0, 2.0};
        double best = cost(start, p, pts);
        for (int iter = 0; iter < 20000; iter++) {
            boolean improved = false;
            for (int i = 0; i < 4; i++) {
                if (!on[i]) continue;
                for (int dir = -1; dir <= 1; dir += 2) {
                    double[] q = p.clone();
                    q[i] += dir * step[i];
                    double cq = cost(start, q, pts);
                    if (cq < best - 1e-12) {
                        p = q;
                        best = cq;
                        improved = true;
                    }
                }
            }
            if (!improved) {
                boolean tiny = true;
                for (int i = 0; i < 4; i++) {
                    step[i] *= 0.5;
                    if (step[i] > 1e-5) tiny = false;
                }
                if (tiny) break;
            }
        }
        return apply(start, p);
    }

    private static Config apply(Config base, double[] p) {
        Config c = base.copy();
        c.cameraPitchDownDeg = p[0];
        c.cameraHeightIn = p[1];
        c.cameraYawDeg = p[2];
        c.cameraOffsetForwardIn = p[3];
        return c;
    }

    private static double cost(Config base, double[] p, double[][] pts) {
        if (p[0] < 0 || p[0] > 89 || p[1] < 0.5 || Math.abs(p[2]) > 45 || Math.abs(p[3]) > 40) return 1e9;
        return rmsErrorIn(apply(base, p), pts);
    }
}