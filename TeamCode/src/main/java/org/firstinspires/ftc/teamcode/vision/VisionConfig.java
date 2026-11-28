package org.firstinspires.ftc.teamcode.vision;

public final class VisionConfig {
    private VisionConfig() {}

    // Camera
    public static String WEBCAM_NAME = "Webcam 1";
    public static int WIDTH = 640;
    public static int HEIGHT = 480;
    public static boolean USE_MJPEG = true;
    public static boolean LIVE_VIEW = true;
    public static double EXPOSURE_MS = 6.0;
    public static int GAIN = -1;

    // Camera mounting
    public static double CAM_HEIGHT_IN = 15.0;
    public static double CAM_PITCH_DEG = 24.67;   // positive = nose UP from horizontal

    // Detector
    public static float DECIMATION = 2.0f;
    public static int THREADS = 3;

    // Calibration
    public static double FX = 0, FY = 0, CX = 0, CY = 0;

    // Filtering
    public static double MIN_DECISION_MARGIN = 20.0;
    public static double STALE_MS = 250.0;
    public static double CLUSTER_CONSISTENCY_IN = 2.0;
    public static double CLUSTER_HEADING_DEG = 15.0;
    public static double CROSS_CELL_DY_MAX_IN = 50.0;
    public static double SMOOTHING_ALPHA = 0.3;
}