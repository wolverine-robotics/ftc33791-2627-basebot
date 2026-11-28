package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.teamcode.vision.VisionConfig;
import org.firstinspires.ftc.teamcode.vision.FieldMap.ClusterPose;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

public final class Larplight {

    public static final class Cluster {
        public int cellIndex;
        public double tx, ty, rangeIn;
        public double robotX, robotY, robotHeading;
        public double percentFound;
        public boolean upright;
        public boolean valid;
        public String note = "";
    }

    public static final class BotPose {
        public double x, y, heading;
        public boolean valid;
        public int clusterCount;
        public double timestamp;
    }

    public enum CellOrientation { UP, DOWN }

    private static final int[] OPPOSITE_CELL = {1, 0, 3, 2};
    private final CellOrientation[] lastKnownState = new CellOrientation[4];
    private final int[] cellOrientationConfidence = new int[4];
    private static final int CONFIDENCE_MAX = 3;

    public Larplight(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        resetCellStates();
    }

    public void resetCellStates() {
        lastKnownState[0] = CellOrientation.UP;
        lastKnownState[1] = CellOrientation.DOWN;
        lastKnownState[2] = CellOrientation.DOWN;
        lastKnownState[3] = CellOrientation.UP;
        for (int i = 0; i < 4; i++) cellOrientationConfidence[i] = CONFIDENCE_MAX;
    }

    public boolean hasTarget;
    public int tid = -1;
    public double tx, ty, rangeIn;
    public final List<Cluster> clusters = new ArrayList<>();
    public final BotPose botPose = new BotPose();
    public double fps;
    public long frames;

    private final HardwareMap hardwareMap;
    private VisionPortal portal;
    private AprilTagProcessor processor;
    private long lastFrameNanos;
    private boolean streaming;
    private String note = "";
    private double exposureMs = Double.NaN;
    private int gain = -1;

    public void init() {
        close();

        AprilTagProcessor.Builder pb = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setNumThreads(VisionConfig.THREADS)
                .setDrawAxes(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true);

        if (VisionConfig.FX > 0 && VisionConfig.FY > 0) {
            pb.setLensIntrinsics(VisionConfig.FX, VisionConfig.FY, VisionConfig.CX, VisionConfig.CY);
        }
        processor = pb.build();
        processor.setDecimation(VisionConfig.DECIMATION);

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, VisionConfig.WEBCAM_NAME))
                .setCameraResolution(new Size(VisionConfig.WIDTH, VisionConfig.HEIGHT))
                .setStreamFormat(VisionConfig.USE_MJPEG ? VisionPortal.StreamFormat.MJPEG : VisionPortal.StreamFormat.YUY2)
                .enableLiveView(VisionConfig.LIVE_VIEW)
                .addProcessor(processor)
                .build();

        long end = System.currentTimeMillis() + 6000;
        while (System.currentTimeMillis() < end && portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            try { Thread.sleep(20); } catch (InterruptedException e) { Thread.currentThread().interrupt(); break; }
        }
        streaming = portal.getCameraState() == VisionPortal.CameraState.STREAMING;
        lastFrameNanos = System.nanoTime();
        applyControls();
    }

    public void close() {
        if (portal != null) {
            try { portal.close(); } catch (RuntimeException ignored) { }
        }
        portal = null;
        processor = null;
        streaming = false;
        clearTarget();
    }

    private void applyControls() {
        if (!streaming) {
            note = "camera not STREAMING";
            return;
        }
        try {
            ExposureControl ec = portal.getCameraControl(ExposureControl.class);
            if (ec != null && ec.isExposureSupported()) {
                if (VisionConfig.EXPOSURE_MS > 0) setExposureMs(VisionConfig.EXPOSURE_MS);
                else exposureMs = ec.getExposure(TimeUnit.MILLISECONDS);
            } else note = "exposure control not supported";
        } catch (RuntimeException e) { note = "exposure error: " + e.getMessage(); }
        try {
            GainControl gc = portal.getCameraControl(GainControl.class);
            if (gc != null) {
                gain = gc.getGain();
                if (VisionConfig.GAIN >= 0) setGain(VisionConfig.GAIN);
            } else note += "; gain control not supported";
        } catch (RuntimeException e) { note += "; gain error: " + e.getMessage(); }
    }

    public void update() {
        if (processor == null || portal == null) return;
        fps = portal.getFps();
        long now = System.nanoTime();
        List<AprilTagDetection> allDetections = processor.getDetections();
        if (allDetections != null) {
            List<AprilTagClusterDetection> fresh = new ArrayList<>();
            for (AprilTagDetection d : allDetections) {
                if (d instanceof AprilTagClusterDetection) fresh.add((AprilTagClusterDetection) d);
            }
            if (!fresh.isEmpty()) {
                lastFrameNanos = now;
                frames++;
                process(fresh);
            }
        } else if ((now - lastFrameNanos) / 1e6 > VisionConfig.STALE_MS) {
            clearTarget();
        }
    }

    private void process(List<AprilTagClusterDetection> detections) {
        clusters.clear();
        botPose.valid = false;
        botPose.clusterCount = 0;
        botPose.timestamp = System.currentTimeMillis();

        double pitchRad = Math.toRadians(-VisionConfig.CAM_PITCH_DEG);
        double cosP = Math.cos(pitchRad);
        double sinP = Math.sin(pitchRad);
        double camHeight = VisionConfig.CAM_HEIGHT_IN;

        class PoseEst {
            final int cellIndex;
            final double robotX, robotY, robotHeading;
            final double weight;
            final Cluster clusterRef;
            final AprilTagClusterDetection det;

            PoseEst(int cellIndex, double robotX, double robotY, double robotHeading,
                    double weight, Cluster clusterRef, AprilTagClusterDetection det) {
                this.cellIndex = cellIndex;
                this.robotX = robotX;
                this.robotY = robotY;
                this.robotHeading = robotHeading;
                this.weight = weight;
                this.clusterRef = clusterRef;
                this.det = det;
            }
        }
        List<PoseEst> allPoses = new ArrayList<>();

        for (AprilTagClusterDetection d : detections) {
            Cluster c = new Cluster();
            int cellIdx = -1;
            if (d.metadata != null) {
                try {
                    cellIdx = (int) d.metadata.getClass().getField("clusterIndex").get(d.metadata);
                } catch (Exception ignored) {
                    try { cellIdx = (int) d.metadata.getClass().getField("cluster_id").get(d.metadata); } catch (Exception ignored2) { }
                }
            }
            c.cellIndex = cellIdx;

            c.tx = d.ftcPose.x;
            c.ty = d.ftcPose.y;
            c.rangeIn = d.ftcPose.range;
            c.percentFound = d.percentClusterFound;
            c.upright = d.ftcPose.roll < 90.0;

            ClusterPose fp = FieldMap.getCluster(c.cellIndex);
            if (fp != null) {
                double tagCamX = d.ftcPose.x;
                double tagCamY = d.ftcPose.y;
                double tagCamZ = d.ftcPose.z;

                double tagRobotX = tagCamX;
                double tagRobotY = tagCamY * cosP - tagCamZ * sinP;
                double tagRobotZ = tagCamY * sinP + tagCamZ * cosP + camHeight;

                double provisionalHeading = normalizeHeading(fp.heading - d.ftcPose.yaw);
                double cosH = Math.cos(Math.toRadians(provisionalHeading));
                double sinH = Math.sin(Math.toRadians(provisionalHeading));
                double offsetFieldX = tagRobotX * cosH - tagRobotY * sinH;
                double offsetFieldY = tagRobotX * sinH + tagRobotY * cosH;

                double robotX = fp.x - offsetFieldX;
                double robotY = fp.y - offsetFieldY;
                double robotHeading = normalizeHeading(fp.heading - d.ftcPose.yaw);

                c.robotX = robotX;
                c.robotY = robotY;
                c.robotHeading = robotHeading;
                c.valid = true;

                double weight = (1.0 / Math.max(d.ftcPose.range, 1.0)) * (d.percentClusterFound + 0.1);
                allPoses.add(new PoseEst(c.cellIndex, robotX, robotY, robotHeading, weight, c, d));
            }
            clusters.add(c);
        }

        List<PoseEst> filteredPoses = new ArrayList<>();
        List<PoseEst>[] byCell = new List[4];
        for (int i = 0; i < 4; i++) byCell[i] = new ArrayList<>();
        for (PoseEst pe : allPoses) byCell[pe.cellIndex].add(pe);

        for (int cell = 0; cell < 4; cell++) {
            List<PoseEst> cellPoses = byCell[cell];
            if (cellPoses.size() < 2) {
                filteredPoses.addAll(cellPoses);
                continue;
            }
            double medX = median(cellPoses.stream().mapToDouble(p -> p.robotX).toArray());
            double medY = median(cellPoses.stream().mapToDouble(p -> p.robotY).toArray());
            double medH = medianAngle(cellPoses.stream().mapToDouble(p -> p.robotHeading).toArray());

            for (PoseEst pe : cellPoses) {
                double dist = Math.hypot(pe.robotX - medX, pe.robotY - medY);
                double hDiff = Math.abs(normalizeHeading(pe.robotHeading - medH));
                if (dist <= VisionConfig.CLUSTER_CONSISTENCY_IN && hDiff <= VisionConfig.CLUSTER_HEADING_DEG) {
                    filteredPoses.add(pe);
                } else {
                    pe.clusterRef.note = "cluster outlier";
                }
            }
        }

        if (filteredPoses.size() >= 2) {
            double redMinY = Double.MAX_VALUE, redMaxY = -Double.MAX_VALUE;
            double blueMinY = Double.MAX_VALUE, blueMaxY = -Double.MAX_VALUE;
            int redCount = 0, blueCount = 0;
            for (PoseEst pe : filteredPoses) {
                if (pe.cellIndex <= 1) { redMinY = Math.min(redMinY, pe.robotY); redMaxY = Math.max(redMaxY, pe.robotY); redCount++; }
                else { blueMinY = Math.min(blueMinY, pe.robotY); blueMaxY = Math.max(blueMaxY, pe.robotY); blueCount++; }
            }
            if (redCount >= 2 && (redMaxY - redMinY) > VisionConfig.CROSS_CELL_DY_MAX_IN) {
                filteredPoses.removeIf(pe -> pe.cellIndex <= 1);
            }
            if (blueCount >= 2 && (blueMaxY - blueMinY) > VisionConfig.CROSS_CELL_DY_MAX_IN) {
                filteredPoses.removeIf(pe -> pe.cellIndex >= 2);
            }
        }

        if (!filteredPoses.isEmpty()) {
            double sumWX = 0, sumWY = 0, sumWSin = 0, sumWCos = 0, sumW = 0;
            for (PoseEst pe : filteredPoses) {
                sumWX += pe.robotX * pe.weight;
                sumWY += pe.robotY * pe.weight;
                sumWSin += Math.sin(Math.toRadians(pe.robotHeading)) * pe.weight;
                sumWCos += Math.cos(Math.toRadians(pe.robotHeading)) * pe.weight;
                sumW += pe.weight;
            }
            double newX = sumWX / sumW;
            double newY = sumWY / sumW;
            double newH = Math.toDegrees(Math.atan2(sumWSin / sumW, sumWCos / sumW));

            double alpha = VisionConfig.SMOOTHING_ALPHA;
            if (botPose.valid) {
                botPose.x = alpha * newX + (1 - alpha) * botPose.x;
                botPose.y = alpha * newY + (1 - alpha) * botPose.y;
                double curSin = Math.sin(Math.toRadians(botPose.heading));
                double curCos = Math.cos(Math.toRadians(botPose.heading));
                double newSin = Math.sin(Math.toRadians(newH));
                double newCos = Math.cos(Math.toRadians(newH));
                double blendSin = alpha * newSin + (1 - alpha) * curSin;
                double blendCos = alpha * newCos + (1 - alpha) * curCos;
                botPose.heading = normalizeHeading(Math.toDegrees(Math.atan2(blendSin, blendCos)));
                botPose.x = alpha * newX + (1 - alpha) * botPose.x;
                botPose.y = alpha * newY + (1 - alpha) * botPose.y;
            } else {
                botPose.x = newX;
                botPose.y = newY;
                botPose.heading = newH;
            }
            botPose.valid = true;
            botPose.clusterCount = filteredPoses.size();
        }

        updateCellStates(detections);

        Cluster chosen = null;
        for (Cluster c : clusters) {
            if (!c.valid) continue;
            if (chosen == null || c.rangeIn < chosen.rangeIn) chosen = c;
        }
        if (chosen == null) { clearTarget(); return; }

        hasTarget = true;
        tid = chosen.cellIndex;
        tx = chosen.tx; ty = chosen.ty; rangeIn = chosen.rangeIn;
    }

    private void clearTarget() {
        hasTarget = false;
        tid = -1;
        tx = 0; ty = 0; rangeIn = 0;
        botPose.valid = false;
        botPose.clusterCount = 0;
    }

    // ---- Cell state tracking with hysteresis ----

    private void updateCellStates(List<AprilTagClusterDetection> detections) {
        boolean[] cellVisible = new boolean[4];
        CellOrientation[] detectedOrientation = new CellOrientation[4];

        for (Cluster c : clusters) {
            if (!c.valid) continue;
            cellVisible[c.cellIndex] = true;
            detectedOrientation[c.cellIndex] = c.upright ? CellOrientation.UP : CellOrientation.DOWN;
        }

        for (int cell = 0; cell < 4; cell++) {
            if (cellVisible[cell] && detectedOrientation[cell] != null) {
                if (lastKnownState[cell] == detectedOrientation[cell]) {
                    cellOrientationConfidence[cell] = CONFIDENCE_MAX;
                } else {
                    cellOrientationConfidence[cell]--;
                    if (cellOrientationConfidence[cell] <= 0) {
                        lastKnownState[cell] = detectedOrientation[cell];
                        cellOrientationConfidence[cell] = CONFIDENCE_MAX;
                    }
                }
            } else if (!cellVisible[cell]) {
                int opp = OPPOSITE_CELL[cell];
                if (cellVisible[opp] && detectedOrientation[opp] != null) {
                    CellOrientation inferred = (detectedOrientation[opp] == CellOrientation.UP) ? CellOrientation.DOWN : CellOrientation.UP;
                    if (lastKnownState[cell] == inferred) {
                        cellOrientationConfidence[cell] = CONFIDENCE_MAX;
                    } else {
                        cellOrientationConfidence[cell]--;
                        if (cellOrientationConfidence[cell] <= 0) {
                            lastKnownState[cell] = inferred;
                            cellOrientationConfidence[cell] = CONFIDENCE_MAX;
                        }
                    }
                }
            }
        }
    }

    // ---- Public API ----

    /** @return UP if cell is scorable (upright), DOWN if tilted away, or last known state */
    public CellOrientation getCellOrientation(int cellIndex) {
        if (cellIndex < 0 || cellIndex >= 4) return CellOrientation.DOWN;
        return lastKnownState[cellIndex];
    }

    /** @param tagIds the 4 tag IDs on a cell (e.g., {0,1,2,3} for red far) */
    public CellOrientation getCellOrientationByTags(int... tagIds) {
        if (tagIds == null || tagIds.length == 0) return CellOrientation.DOWN;
        int cell = -1;
        for (int id : tagIds) {
            int c = FieldMap.getClusterIndex(id);
            if (c >= 0) { cell = c; break; }
        }
        if (cell < 0) return CellOrientation.DOWN;
        return lastKnownState[cell];
    }

    public CellOrientation[] getAllCellOrientations() {
        return lastKnownState.clone();
    }

    /** @return true if the specified cell is upright (scorable) */
    public boolean isCellUpright(int cellIndex) {
        return getCellOrientation(cellIndex) == CellOrientation.UP;
    }

    /** @return the best upright (scorable) cluster for scoring, or null */
    public Cluster getBestScoringCluster() {
        Cluster best = null;
        for (Cluster c : clusters) {
            if (!c.valid || !c.upright) continue;
            if (best == null || c.percentFound > best.percentFound) best = c;
        }
        return best;
    }

    /** @return true if any cell is currently upright (scorable) */
    public boolean hasAnyUprightCell() {
        for (Cluster c : clusters) if (c.valid && c.upright) return true;
        return false;
    }

    /** @return list of cell indices that are upright (scorable) */
    public int[] getUprightCells() {
        int count = 0;
        for (Cluster c : clusters) if (c.valid && c.upright) count++;
        int[] result = new int[count];
        int idx = 0;
        for (Cluster c : clusters) if (c.valid && c.upright) result[idx++] = c.cellIndex;
        return result;
    }

    /** @return robot pose relative to a specific cell (x forward, y right, heading), or null if not visible */
    public double[] getRobotPoseToCell(int cellIndex) {
        Cluster c = getCluster(cellIndex);
        if (c == null || !c.valid) return null;
        return new double[]{c.robotX, c.robotY, c.robotHeading};
    }

    /** @return true if robot is aligned with cell within tolerances (degrees) */
    public boolean isAlignedWithCell(int cellIndex, double txTolDeg, double tyTolDeg) {
        Cluster c = getCluster(cellIndex);
        if (c == null || !c.valid) return false;
        return Math.abs(c.tx) <= txTolDeg && Math.abs(c.ty) <= tyTolDeg;
    }

    /** @return heading error (deg) to face cell directly, or Double.NaN if not visible */
    public double getCellAngleError(int cellIndex) {
        Cluster c = getCluster(cellIndex);
        if (c == null || !c.valid) return Double.NaN;
        return normalizeHeading(-c.robotHeading);
    }

    /** @return list of all valid clusters */
    public Cluster[] getAllValidClusters() {
        int count = 0;
        for (Cluster c : clusters) if (c.valid) count++;
        Cluster[] result = new Cluster[count];
        int idx = 0;
        for (Cluster c : clusters) if (c.valid) result[idx++] = c;
        return result;
    }

    /** @return whether we have a fresh frame this cycle */
    public boolean hasFreshFrame() {
        return processor != null && processor.getDetections() != null;
    }

    /** @return the nearest valid cluster for aiming, or null */
    public Cluster getNearestCluster() {
        Cluster best = null;
        for (Cluster c : clusters) {
            if (!c.valid) continue;
            if (best == null || c.rangeIn < best.rangeIn) best = c;
        }
        return best;
    }

    /** @return cluster for specific cell index, or null if not visible */
    public Cluster getCluster(int cellIndex) {
        for (Cluster c : clusters) if (c.cellIndex == cellIndex && c.valid) return c;
        return null;
    }

    /** @return distance to cell in inches, or -1 if not visible */
    public double getCellDistance(int cellIndex) {
        Cluster c = getCluster(cellIndex);
        return c != null ? c.rangeIn : -1;
    }

    // ---- Camera controls ----

    public void setExposureMs(double ms) {
        if (!streaming || portal == null) return;
        try {
            ExposureControl ec = portal.getCameraControl(ExposureControl.class);
            if (ec == null || !ec.isExposureSupported()) return;
            if (ec.getMode() != ExposureControl.Mode.Manual) ec.setMode(ExposureControl.Mode.Manual);
            long v = Math.max(ec.getMinExposure(TimeUnit.MILLISECONDS),
                    Math.min(ec.getMaxExposure(TimeUnit.MILLISECONDS), Math.round(ms)));
            ec.setExposure(v, TimeUnit.MILLISECONDS);
            exposureMs = v;
        } catch (RuntimeException e) { note = "setExposure failed: " + e.getMessage(); }
    }

    public void adjustExposureMs(double delta) {
        setExposureMs((Double.isNaN(exposureMs) ? VisionConfig.EXPOSURE_MS : exposureMs) + delta);
    }

    public void setGain(int g) {
        if (!streaming || portal == null) return;
        try {
            GainControl gc = portal.getCameraControl(GainControl.class);
            if (gc == null) return;
            int v = Math.max(gc.getMinGain(), Math.min(gc.getMaxGain(), g));
            gc.setGain(v);
            gain = v;
        } catch (RuntimeException e) { note = "setGain failed: " + e.getMessage(); }
    }

    public void adjustGain(int delta) {
        setGain((gain < 0 ? 0 : gain) + delta);
    }

    public void saveFrame(String name) {
        if (portal != null) portal.saveNextFrameRaw(name);
    }

    public void addTelemetry(Telemetry t) {
        t.addLine(String.format(Locale.US, "CAM %s %dx%d %s fps=%.1f exp=%s gain=%s",
                portal == null ? "NOT_BUILT" : portal.getCameraState(),
                VisionConfig.WIDTH, VisionConfig.HEIGHT, VisionConfig.USE_MJPEG ? "MJPEG" : "YUY2", fps,
                Double.isNaN(exposureMs) ? "?" : String.format(Locale.US, "%.0f", exposureMs),
                gain < 0 ? "?" : gain));
        t.addLine(String.format(Locale.US, "calib %s frames %d",
                (VisionConfig.FX > 0 ? "custom" : "default"), frames));
        if (!note.isEmpty()) t.addLine("note: " + note);

        t.addLine("OUTPUTS");
        t.addData("hasTarget", hasTarget);
        t.addData("tid", tid);
        t.addData("tx", String.format(Locale.US, "%+.2f", tx));
        t.addData("ty", String.format(Locale.US, "%+.2f", ty));
        t.addData("range", String.format(Locale.US, "%.2f", rangeIn));
        t.addData("botPose", botPose.valid);
        t.addData("x", String.format(Locale.US, "%.2f", botPose.x));
        t.addData("y", String.format(Locale.US, "%.2f", botPose.y));
        t.addData("heading", String.format(Locale.US, "%.2f", botPose.heading));
        t.addData("clusters", botPose.clusterCount);

        t.addLine("CLUSTERS");
        t.addLine("cell tx ty range %found upright note");
        for (Cluster c : clusters) {
            t.addLine(String.format(Locale.US, "%-4d %+7.2f %+7.2f %7.2f %5.1f%% %s %s",
                    c.cellIndex, c.tx, c.ty, c.rangeIn, c.percentFound * 100,
                    c.upright ? "UP" : "DOWN", c.note));
        }
        if (clusters.isEmpty()) t.addLine("(no clusters)");
    }

    // ---- Helpers ----

    private static double normalizeHeading(double h) {
        while (h <= -180) h += 360;
        while (h > 180) h -= 360;
        return h;
    }

    private static double median(double[] arr) {
        if (arr.length == 0) return 0;
        java.util.Arrays.sort(arr);
        return arr.length % 2 == 0 ? (arr[arr.length/2 - 1] + arr[arr.length/2]) / 2.0 : arr[arr.length/2];
    }

    private static double medianAngle(double[] arr) {
        if (arr.length == 0) return 0;
        double sumSin = 0, sumCos = 0;
        for (double a : arr) {
            double rad = Math.toRadians(a);
            sumSin += Math.sin(rad);
            sumCos += Math.cos(rad);
        }
        return Math.toDegrees(Math.atan2(sumSin / arr.length, sumCos / arr.length));
    }

    private static class PoseEst {
        final int cellIndex;
        final double robotX, robotY, robotHeading;
        final double weight;
        final Cluster clusterRef;
        final AprilTagClusterDetection det;

        PoseEst(int cellIndex, double robotX, double robotY, double robotHeading,
                double weight, Cluster clusterRef, AprilTagClusterDetection det) {
            this.cellIndex = cellIndex;
            this.robotX = robotX;
            this.robotY = robotY;
            this.robotHeading = robotHeading;
            this.weight = weight;
            this.clusterRef = clusterRef;
            this.det = det;
        }
    }
}