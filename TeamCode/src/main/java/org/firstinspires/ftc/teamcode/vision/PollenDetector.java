package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

/**
 * PollenDetector - asks the Limelight for pollen and reports each one's X/Y distance (inches).
 * Uses PollenDistanceConversion for the math.
 *
 * NOT ALWAYS ON: constructing this does nothing to the camera. The Limelight is only started
 * when you call start() (or look()), and stopped with stop(). Create it where you need it and let
 * it go when you're done.
 *
 * Output frame: X = right (+), Y = forward (+), inches. Nearest pollen is index 0.
 *
 * ---- USE 1: ONE LOOK (e.g. at the start of an auto routine) --------------------------------
 *     PollenDetector pollen = new PollenDetector(hardwareMap);
 *     if (pollen.look(1000)) {                  // starts the camera, waits up to 1 s, stops it again
 *         double x = pollen.nearestX();         // inches right of the output origin
 *         double y = pollen.nearestY();         // inches forward
 *         // ... drive to (x, y) ...
 *     }
 *
 * ---- USE 2: CONSTANT LOOKING (inside a loop that needs fresh data every cycle) -----------------
 *     PollenDetector pollen = new PollenDetector(hardwareMap).start();
 *     while (opModeIsActive()) {
 *         if (pollen.update()) {                // true = at least one pollen seen this cycle
 *             double x = pollen.nearestX(), y = pollen.nearestY();
 *             // ... steer toward it ...
 *         }
 *     }
 *     pollen.stop();
 *
 * Flicker handling: if the last look saw pollen but this one doesn't, update() re-polls up to
 * repollAttempts times (one camera frame apart) before it reports "none". That costs up to
 * repollAttempts * (1000 / pollRateHz) ms once, when a pollen really leaves the view.
 */
public class PollenDetector {

    /** Default Limelight result polling rate (Hz). Change per-instance with setPollRateHz(). */
    public static final int DEFAULT_POLL_RATE_HZ = 30;
    /** Most pollen tracked at once (matches the pipeline's max group count). */
    public static final int MAX_POLLEN = 8;

    private final HardwareMap hardwareMap;
    private final String deviceName;
    private final PollenDistanceConversion conversion = new PollenDistanceConversion();
    private final PollenDistanceConversion.Buffer found = new PollenDistanceConversion.Buffer(MAX_POLLEN);

    private int pipeline = 0;                      // Pollen_Detector pipeline slot on the Limelight
    private int pollRateHz = DEFAULT_POLL_RATE_HZ;
    private int repollAttempts = 2;
    private long maxStalenessMs = 100;             // ignore results older than this

    private Limelight3A limelight;                 // null until start()
    private boolean running = false;
    private boolean hadPollen = false;

    public PollenDetector(HardwareMap hardwareMap) {
        this(hardwareMap, "limelight");
    }

    /** deviceName = the Limelight's name in the Robot Configuration. */
    public PollenDetector(HardwareMap hardwareMap, String deviceName) {
        this.hardwareMap = hardwareMap;
        this.deviceName = deviceName;
    }

    // ---------------- settings (chainable; call before start()) ----------------
    public PollenDetector setPipeline(int pipelineIndex) {
        this.pipeline = pipelineIndex;
        return this;
    }

    /** How often the Limelight is asked for results. Default 30. Applied at start(). */
    public PollenDetector setPollRateHz(int hz) {
        this.pollRateHz = Math.max(1, hz);
        return this;
    }

    /** Re-polls after a "none" that follows a successful look (flicker guard). 0 turns it off. */
    public PollenDetector setRepollAttempts(int attempts) {
        this.repollAttempts = Math.max(0, attempts);
        return this;
    }

    /** Results older than this (ms) are treated as "no data". */
    public PollenDetector setMaxStalenessMs(long ms) {
        this.maxStalenessMs = ms;
        return this;
    }

    /** Calibration values (height, pitch, yaw, offsets, ball size) used for the distance math. */
    public PollenDistanceConversion.Config config() {
        return conversion.config();
    }

    // ---------------- lifecycle ----------------
    /** Start the Limelight. Safe to call twice. */
    public PollenDetector start() {
        if (running) return this;
        if (limelight == null) limelight = hardwareMap.get(Limelight3A.class, deviceName);
        limelight.pipelineSwitch(pipeline);
        limelight.setPollRateHz(pollRateHz);
        limelight.start();
        found.clear();
        hadPollen = false;
        running = true;
        return this;
    }

    /** Stop the Limelight. The last results stay readable. */
    public void stop() {
        if (running) {
            limelight.stop();
            running = false;
        }
    }

    public boolean isRunning() {
        return running;
    }

    // ---------------- looking ----------------
    /**
     * One look, for loops. Refreshes the results and returns true if at least one pollen is visible.
     * Returns false immediately if start() has not been called.
     */
    public boolean update() {
        if (!running) return false;
        boolean ok = readOnce();
        if (!ok && hadPollen) {                         // flicker guard: pollen just vanished, look again
            for (int i = 0; i < repollAttempts && !ok; i++) {
                if (!sleepMs(frameMs())) break;
                ok = readOnce();
            }
        }
        hadPollen = ok;
        return ok;
    }

    /**
     * One-shot look. Starts the Limelight if it isn't running, keeps looking (one frame apart) until a
     * pollen is seen or timeoutMs passes, then stops the Limelight again if it started it.
     * Allow at least ~500 ms when the camera has to start up. Results stay readable afterwards.
     */
    public boolean look(long timeoutMs) {
        boolean startedHere = !running;
        if (startedHere) start();
        long deadline = System.nanoTime() + timeoutMs * 1000000L;
        boolean ok;
        while (true) {
            ok = readOnce();
            if (ok || System.nanoTime() >= deadline) break;
            if (!sleepMs(frameMs())) break;
        }
        hadPollen = ok;
        if (startedHere) stop();
        return ok;
    }

    // ---------------- results (valid after update() / look()) ----------------
    public boolean hasPollen() {
        return found.count > 0;
    }

    public int count() {
        return found.count;
    }

    /** X of pollen i (inches right), nearest = 0. NaN if there is no pollen i. */
    public double x(int i) {
        return (i >= 0 && i < found.count) ? found.x[i] : Double.NaN;
    }

    /** Y of pollen i (inches forward), nearest = 0. NaN if there is no pollen i. */
    public double y(int i) {
        return (i >= 0 && i < found.count) ? found.y[i] : Double.NaN;
    }

    /** Straight-line floor distance to pollen i from the output origin. NaN if none. */
    public double rangeIn(int i) {
        return (i >= 0 && i < found.count) ? found.rangeIn[i] : Double.NaN;
    }

    public double nearestX() {
        return x(0);
    }

    public double nearestY() {
        return y(0);
    }

    public double nearestRangeIn() {
        return rangeIn(0);
    }

    /** Angle to the nearest pollen from straight ahead, degrees, right = positive. NaN if none. */
    public double nearestBearingDeg() {
        return found.count > 0 ? Math.toDegrees(Math.atan2(found.x[0], found.y[0])) : Double.NaN;
    }

    /** Raw results, nearest first (x[], y[], rangeIn[], count). */
    public PollenDistanceConversion.Buffer results() {
        return found;
    }

    // ---------------- internals ----------------
    private boolean readOnce() {
        found.clear();
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return false;
        if (result.getStaleness() > maxStalenessMs) return false;
        List<LLResultTypes.ColorResult> blobs = result.getColorResults();
        for (int i = 0; i < blobs.size(); i++) {      // index loop: no iterator allocated
            LLResultTypes.ColorResult c = blobs.get(i);
            conversion.addTo(found, c.getTargetXDegrees(), c.getTargetYDegrees());
        }
        return found.count > 0;
    }

    private long frameMs() {
        return Math.max(1, 1000 / pollRateHz);
    }

    private static boolean sleepMs(long ms) {
        try {
            Thread.sleep(ms);
            return true;
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();   // opmode is stopping: let the caller bail out
            return false;
        }
    }
}