package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "FUNBLUEQUICKMAINSIDE", group = "Autonomous")
public class FUNBLUEQUICKMAINSIDE extends OpMode {

    // ---- Blue side field values ----
    private static final double START_HEADING_DEG = 270;
    private static final double FACE_HEADING_DEG = 0;     // facing the right wall (flower)
    private static final int APPROACH_SIGN = -1;          // robot comes in from -x of the flower

    private final Pose startPose = new Pose(104.8712, 135.6372, Math.toRadians(START_HEADING_DEG));
    private final Pose endPose = new Pose(130.1667, 45.7763, Math.toRadians(START_HEADING_DEG));
    private final Pose flowerPose = new Pose(134.01, 93.98);
    private final Pose control1 = new Pose(80, 110);
    private final Pose control2 = new Pose(125, 70);

    // ---- Tuning ----
    private static final int BANGS = 3;
    private static final double IMPACT_STANDOFF_IN = 9;   // robot CENTER to flower at impact (about half robot length)
    private static final double BACKOFF_IN = 20;          // robot center to flower when backed off
    private static final double EXIT_SWING_IN = 30;       // how far the exit curve swings away from the wall/flower (x direction)
    private static final int SPINS_IN = 6;                // full spins on the way in
    private static final int SPINS_OUT = 4;               // full spins on the way to park
    private static final double STEP_DEG = 45;
    private static final long FLASH_MS = 120;
    private static final boolean USE_BUILTIN_STROBE = false;

    private Follower follower;
    private RevBlinkinLedDriver leds;
    private final ElapsedTime flashTimer = new ElapsedTime();
    private boolean ledOn = false;
    private int pathState;
    private PathChain fullRoute;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        leds = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        Pose approach = new Pose(flowerPose.getX() + APPROACH_SIGN * BACKOFF_IN, flowerPose.getY());
        Pose impact = new Pose(flowerPose.getX() + APPROACH_SIGN * IMPACT_STANDOFF_IN, flowerPose.getY());
        double faceRad = Math.toRadians(FACE_HEADING_DEG);

        double inTotal = 360 * SPINS_IN + ccwDelta(START_HEADING_DEG, FACE_HEADING_DEG);
        double outTotal = 360 * SPINS_OUT + ccwDelta(FACE_HEADING_DEG, START_HEADING_DEG);

        PathBuilder b = follower.pathBuilder();

        // 1) Spin in toward the flower, ending up facing it
        b.addPath(new BezierCurve(startPose, control1, control2, approach))
                .setHeadingInterpolation(spin(START_HEADING_DEG, inTotal));

        // 2) Bang, back off, bang, back off, bang
        for (int i = 0; i < BANGS; i++) {
            b.addPath(new BezierLine(approach, impact))
                    .setConstantHeadingInterpolation(faceRad);
            if (i < BANGS - 1) {
                b.addPath(new BezierLine(impact, approach))
                        .setConstantHeadingInterpolation(faceRad);
            }
        }

        // 3) Swing away from the flower/wall, then spin back in to park (ending at the original heading)
        Pose exitC1 = new Pose(impact.getX() + APPROACH_SIGN * EXIT_SWING_IN, impact.getY());
        Pose exitC2 = new Pose(impact.getX() + APPROACH_SIGN * EXIT_SWING_IN, endPose.getY());
        b.addPath(new BezierCurve(impact, exitC1, exitC2, endPose))
                .setHeadingInterpolation(spin(FACE_HEADING_DEG, outTotal));

        fullRoute = b.build();
        flashTimer.reset();
    }

    @Override
    public void init_loop() {
        updateErrorLeds();
        telemetry.addData("LED on", ledOn);
        telemetry.update();
    }

    @Override
    public void start() {
        pathState = 0;
        flashTimer.reset();
    }

    @Override
    public void loop() {
        follower.update();
        updateErrorLeds();

        switch (pathState) {
            case 0:
                follower.followPath(fullRoute);
                pathState = 1;
                break;
            case 1:
                if (!follower.isBusy()) pathState = 2;
                break;
            case 2:
                break;
        }

        telemetry.addData("LED on", ledOn);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("busy", follower.isBusy());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (leds != null) leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }

    // ---- Helpers ----
    private static double norm(double deg) {
        return ((deg % 360) + 360) % 360;
    }

    private static double ccwDelta(double fromDeg, double toDeg) {
        return norm(toDeg - fromDeg);
    }

    /** Counterclockwise spin of totalDeg degrees, built from 45 degree nodes so it never takes the short way. */
    private HeadingInterpolator spin(double fromDeg, double totalDeg) {
        int steps = Math.max(1, (int) Math.round(totalDeg / STEP_DEG));
        double span = 1.0 / steps;
        HeadingInterpolator.PiecewiseNode[] nodes = new HeadingInterpolator.PiecewiseNode[steps];
        for (int i = 0; i < steps; i++) {
            double a = Math.toRadians(norm(fromDeg + STEP_DEG * i));
            double b = Math.toRadians(norm(fromDeg + STEP_DEG * (i + 1)));
            nodes[i] = new HeadingInterpolator.PiecewiseNode(
                    i * span, (i + 1) * span, HeadingInterpolator.linear(a, b));
        }
        return HeadingInterpolator.piecewise(nodes);
    }

    /** Flashes red/off like an error light. Writes the pattern every loop. */
    private void updateErrorLeds() {
        if (USE_BUILTIN_STROBE) {
            leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
            ledOn = true;
            return;
        }
        if (flashTimer.milliseconds() >= FLASH_MS) {
            ledOn = !ledOn;
            flashTimer.reset();
        }
        leds.setPattern(ledOn
                ? RevBlinkinLedDriver.BlinkinPattern.RED
                : RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }
}