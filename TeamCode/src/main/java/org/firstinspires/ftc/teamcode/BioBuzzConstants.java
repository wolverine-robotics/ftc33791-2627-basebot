package org.firstinspires.ftc.teamcode;

/**
 * Device names and tunables for 2627 BIOBUZZ.
 */
public final class BioBuzzConstants {

    private BioBuzzConstants() {}

    public static final double HIVE_RED_X = 57.54;
    public static final double HIVE_RED_Y = 52.89;
    public static final double HIVE_BLUE_X = 86.46;
    public static final double HIVE_BLUE_Y = 91.11;

    public static final String FL = "fl";
    public static final String FR = "fr";
    public static final String BL = "bl";
    public static final String BR = "br";
    public static final String INTAKE = "intake";
    public static final String TRANSFER = "transfer";
    public static final String SHOOTER_L = "shooterL";
    public static final String SHOOTER_R = "shooterR";
    public static final String PINPOINT = "pinpoint";
    public static final String WEBCAM = "arducam";
    public static final String LIMELIGHT = "limelight";

    public static final String HOOD1 = "hood1";
    public static final String HOOD2 = "hood2";
    public static final String TURRET1 = "turret1";
    public static final String TURRET2 = "turret2";
    public static final String TURRET3 = "turret3";
    public static final String TURRET4 = "turret4";
    public static final String DEPOL = "depol";
    public static final String BLOCKER = "blocker";
    public static final String BREAK_BEAM_FRONT = "breakBeamFront";
    public static final String PRISM = "prism";

    public static final boolean BREAK_BEAM_TRIPPED_WHEN_HIGH = false;
    public static final double DEPOL_RETRACTED_POS = 0.15;
    public static final double DEPOL_EXTENDED_POS = 0.85;
    public static final double BLOCKER_OPEN_POS = 0.25;
    public static final double BLOCKER_CLOSED_POS = 0.75;

    public static final int LIMELIGHT_PIPELINE_POLLEN = 0;
    public static final int LIMELIGHT_PIPELINE_NECTAR = 1;
    public static final double LL_DRIVE_KP_TX = 0.035;
    public static final double LL_DRIVE_KP_TY = 0.028;
    public static final double LL_ALIGN_TX_TOL = 3.0;
    public static final double LL_ALIGN_TY_TOL = 4.0;
    public static final double LL_MIN_TARGET_AREA = 0.02;
    public static final double AUTO_DRIVE_MAX_POWER = 0.45;
    public static final double AUTO_SEARCH_TIMEOUT_SEC = 8.0;
    public static final double AUTO_APPROACH_TIMEOUT_SEC = 6.0;
    public static final double AUTO_INTAKE_SEC = 2.5;

    public static final double[] SHOOTER_DISTANCES_IN = {27, 29.25, 30.2, 31, 32, 34, 35, 46, 47, 48, 9999};
    public static final double[] SHOOTER_VELOCITIES_TICKS_PER_SEC =
            {1055, 1150, 1150, 1125, 1150, 1200, 1200, 1400, 1400, 1400, 1045};

    public static final double PIVOT_MULTIPLIER = 0.8;
    public static final double STRAFE_CORRECTION = 1.1;
    public static final double INTAKE_POWER = 1.0;
    public static final double INTAKE_REVERSE_POWER = -0.7;
    public static final double TRIGGER_THRESHOLD = 0.2;
    public static final int INDEX_STEP = 280;
    public static final double PASSIVE_INDEX_VELOCITY = 20;
    public static final double MAG_DUMP_POWER = 0.9;

    public static final int SHOOTER_TOLERANCE = 50;
    public static int SHOOTER_PLUS = 50;
    public static final double TBH_FULL_POWER = 1.0;
    public static final double TBH_HALF_FACTOR = 0.5;
    public static final double TBH_TOLERANCE = 30.0;
    public static final double TBH_MIN_POWER = 0.2;

    public static final double HOOD_DISTANCE_NEAR_IN = 24.0;
    public static final double HOOD_DISTANCE_FAR_IN = 110.0;
    public static final double HOOD_ANGLE_NEAR_DEG = 32.0;
    public static final double HOOD_ANGLE_FAR_DEG = 48.0;
    public static final double HOOD_SERVO_MIN = 0.15;
    public static final double HOOD_SERVO_MAX = 0.55;
    public static final double TURRET_KP = 0.02;
    public static final double TURRET_MAX_POWER = 0.65;

    /** Pinpoint pod offsets (mm); match {@code pedroPathing.Constants} localizer. */
    public static final double PINPOINT_X_OFFSET_MM = -107.31371;
    public static final double PINPOINT_Y_OFFSET_MM = 0.0;
}