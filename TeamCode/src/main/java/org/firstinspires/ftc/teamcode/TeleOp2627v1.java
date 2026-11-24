package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import android.graphics.Color;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "TeleOp2627v1", group = "TeleOp")
public class TeleOp2627v1 extends LinearOpMode {

    /** Filled by autonomous OpMode; teleop applies with {@code setPosition} (no IMU reset). */
    public static boolean hasEndOfAutoPose;
    public static double endOfAutoXIn;
    public static double endOfAutoYIn;
    public static double endOfAutoHeadingDeg;

    private final ElapsedTime runtime = new ElapsedTime();
    private Gamepad gamepad;

    private GoBildaPinpointDriver pinpoint;
    private DcMotorEx fl, fr, bl, br, intake, transfer, shooterL, shooterR;
    private Servo hood1, hood2;
    private final List<CRServo> turretMotors = new ArrayList<>();
    private DigitalChannel breakBeamFront;
    private NormalizedColorSensor prism;
    private Servo depol, blocker;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private AprilTagDetection latestTag;

    private boolean redAlliance = true;
    private boolean depolOut, blockerClosed;
    private boolean prevFrontBeamBroken;
    private AimSolution lastAim;

    private double shooterTargetVel;
    private double tbhPowerL, tbhPowerR;

    @Override
    public void runOpMode() throws InterruptedException {
        initMotorsAndMechanisms();
        configurePinpoint(pinpoint);
        initAprilTag();

        gamepad = new Gamepad();
        telemetry.addData("Alliance", "RED (touchpad = toggle)");
        if (hasEndOfAutoPose) {
            telemetry.addData("Auto pose", "(%.1f, %.1f) %.1f°",
                    endOfAutoXIn, endOfAutoYIn, endOfAutoHeadingDeg);
        }
        telemetry.addData(">", "Press START");
        telemetry.update();

        waitForStart();
        runtime.reset();

        if (hasEndOfAutoPose) {
            pinpoint.setPosition(new Pose2D(
                    DistanceUnit.INCH, endOfAutoXIn, endOfAutoYIn,
                    AngleUnit.DEGREES, endOfAutoHeadingDeg));
        }

        while (opModeIsActive()) {
            gamepad.copy(gamepad1);
            pinpoint.update();
            updateAprilTag();

            if (gamepad.touchpadWasPressed()) {
                redAlliance = !redAlliance;
            }

            double heading = pinpoint.getHeading(AngleUnit.RADIANS);

            if (gamepad.cross || gamepad.right_bumper) {
                lastAim = solveAim(redAlliance);
                applyAim(lastAim);
                if (gamepad.right_bumper) {
                    shooterTargetVel = predictShooterVelocity(lastAim.distanceInches) + BioBuzzConstants.SHOOTER_PLUS;
                }
            } else {
                stopTurret();
            }

            if (!gamepad.cross) {
                double directionY = -gamepad.left_stick_y;
                double directionX = gamepad.left_stick_x;
                double pivot = gamepad.right_stick_x * BioBuzzConstants.PIVOT_MULTIPLIER;
                fieldCentric(directionY, directionX, pivot, heading);
            } else {
                stopDrive();
            }

            if (gamepad.optionsWasPressed()) {
                pinpoint.resetPosAndIMU();
                sleep(250);
            }
            if (gamepad.shareWasPressed()) {
                applyAprilTagToPinpoint();
            }
            if (gamepad.squareWasPressed() && blocker != null) {
                blockerClosed = !blockerClosed;
                blocker.setPosition(blockerClosed ? BioBuzzConstants.BLOCKER_CLOSED_POS : BioBuzzConstants.BLOCKER_OPEN_POS);
            }
            if (gamepad.yWasPressed() && depol != null) {
                depolOut = !depolOut;
                depol.setPosition(depolOut ? BioBuzzConstants.DEPOL_EXTENDED_POS : BioBuzzConstants.DEPOL_RETRACTED_POS);
            }

            if (gamepad.leftBumperWasPressed()) {
                stopShooter();
            }
            updateShooter();

            handleIntakeIndex();

            telemetry.addData("Alliance", redAlliance ? "RED hive1" : "BLUE hive4");
            if (lastAim != null) {
                telemetry.addData("Shot", "dist %.1f turretErr %.1f hood %.1f",
                        lastAim.distanceInches, lastAim.turretErrorDeg, lastAim.hoodAngleDeg);
            }
            telemetry.addData("Shooter", "tgt %d avg %d", (int) shooterTargetVel, avgShooterVel());
            telemetry.addData("AprilTag", "id %d", latestTag != null ? latestTag.id : -1);
            if (prism != null) {
                float[] hsv = new float[3];
                Color.colorToHSV(prism.getNormalizedColors().toColor(), hsv);
                telemetry.addData("Prism hue", "%.0f", hsv[0]);
            }
            if (breakBeamFront != null) {
                telemetry.addData("Front beam", isFrontBeamBroken() ? "broken" : "clear");
            }
            telemetry.update();
        }

        stopShooter();
        stopTurret();
        visionPortal.close();
    }

    /** Shared with auto — same 4-bar setup as {@code pedroPathing.Constants}. */
    public static void configurePinpoint(GoBildaPinpointDriver pinpoint) {
        pinpoint.setOffsets(
                BioBuzzConstants.PINPOINT_X_OFFSET_MM,
                BioBuzzConstants.PINPOINT_Y_OFFSET_MM,
                DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
    }

    private void initMotorsAndMechanisms() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, BioBuzzConstants.PINPOINT);

        fl = hardwareMap.get(DcMotorEx.class, BioBuzzConstants.FL);
        fr = hardwareMap.get(DcMotorEx.class, BioBuzzConstants.FR);
        bl = hardwareMap.get(DcMotorEx.class, BioBuzzConstants.BL);
        br = hardwareMap.get(DcMotorEx.class, BioBuzzConstants.BR);
        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        intake = hardwareMap.get(DcMotorEx.class, BioBuzzConstants.INTAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        transfer = hardwareMap.get(DcMotorEx.class, BioBuzzConstants.TRANSFER);
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        transfer.setTargetPosition(0);

        shooterL = hardwareMap.get(DcMotorEx.class, BioBuzzConstants.SHOOTER_L);
        shooterR = hardwareMap.get(DcMotorEx.class, BioBuzzConstants.SHOOTER_R);
        shooterL.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterR.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        hood1 = getServo(BioBuzzConstants.HOOD1);
        hood2 = getServo(BioBuzzConstants.HOOD2);
        for (String name : new String[]{
                BioBuzzConstants.TURRET1, BioBuzzConstants.TURRET2,
                BioBuzzConstants.TURRET3, BioBuzzConstants.TURRET4}) {
            if (hardwareMap.getAllNames(CRServo.class).contains(name)) {
                turretMotors.add(hardwareMap.get(CRServo.class, name));
            }
        }
        breakBeamFront = getDigital(BioBuzzConstants.BREAK_BEAM_FRONT);
        if (breakBeamFront != null) {
            breakBeamFront.setMode(DigitalChannel.Mode.INPUT);
        }
        if (hardwareMap.getAllNames(NormalizedColorSensor.class).contains(BioBuzzConstants.PRISM)) {
            prism = hardwareMap.get(NormalizedColorSensor.class, BioBuzzConstants.PRISM);
        }
        depol = getServo(BioBuzzConstants.DEPOL);
        blocker = getServo(BioBuzzConstants.BLOCKER);
    }

    private void initAprilTag() {
        Position camPos = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
        YawPitchRollAngles camOri = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);
        aprilTag = new AprilTagProcessor.Builder().setCameraPose(camPos, camOri).build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, BioBuzzConstants.WEBCAM))
                .addProcessor(aprilTag)
                .build();
    }

    private static class AimSolution {
        double distanceInches, turretErrorDeg, hoodAngleDeg;
    }

    private AimSolution solveAim(boolean red) {
        double rx = pinpoint.getPosition().getX(DistanceUnit.INCH);
        double ry = pinpoint.getPosition().getY(DistanceUnit.INCH);
        double hx = red ? BioBuzzConstants.HIVE_RED_X : BioBuzzConstants.HIVE_BLUE_X;
        double hy = red ? BioBuzzConstants.HIVE_RED_Y : BioBuzzConstants.HIVE_BLUE_Y;
        double dx = hx - rx, dy = hy - ry;
        AimSolution s = new AimSolution();
        s.distanceInches = Math.hypot(dx, dy);
        double bearing = Math.toDegrees(Math.atan2(dy, dx));
        s.turretErrorDeg = AngleUnit.normalizeDegrees(bearing - pinpoint.getHeading(AngleUnit.DEGREES));
        double t = Range.clip((s.distanceInches - BioBuzzConstants.HOOD_DISTANCE_NEAR_IN)
                / (BioBuzzConstants.HOOD_DISTANCE_FAR_IN - BioBuzzConstants.HOOD_DISTANCE_NEAR_IN), 0, 1);
        s.hoodAngleDeg = BioBuzzConstants.HOOD_ANGLE_NEAR_DEG
                + t * (BioBuzzConstants.HOOD_ANGLE_FAR_DEG - BioBuzzConstants.HOOD_ANGLE_NEAR_DEG);
        return s;
    }

    private void applyAim(AimSolution aim) {
        if (hood1 != null && hood2 != null) {
            double span = BioBuzzConstants.HOOD_ANGLE_FAR_DEG - BioBuzzConstants.HOOD_ANGLE_NEAR_DEG;
            double u = Range.clip((aim.hoodAngleDeg - BioBuzzConstants.HOOD_ANGLE_NEAR_DEG) / span, 0, 1);
            double pos = BioBuzzConstants.HOOD_SERVO_MIN + u * (BioBuzzConstants.HOOD_SERVO_MAX - BioBuzzConstants.HOOD_SERVO_MIN);
            hood1.setPosition(pos);
            hood2.setPosition(pos);
        }
        if (!turretMotors.isEmpty()) {
            double power = Range.clip(aim.turretErrorDeg * BioBuzzConstants.TURRET_KP,
                    -BioBuzzConstants.TURRET_MAX_POWER, BioBuzzConstants.TURRET_MAX_POWER);
            if (Math.abs(aim.turretErrorDeg) < 1.5) power = 0;
            for (CRServo t : turretMotors) t.setPower(power);
        }
    }

    private void stopTurret() {
        for (CRServo t : turretMotors) t.setPower(0);
    }

    private static double predictShooterVelocity(double distanceInches) {
        double[] d = BioBuzzConstants.SHOOTER_DISTANCES_IN;
        double[] v = BioBuzzConstants.SHOOTER_VELOCITIES_TICKS_PER_SEC;
        if (distanceInches <= d[0]) return v[0];
        int last = d.length - 1;
        if (distanceInches >= d[last]) return v[last];
        for (int i = 0; i < last; i++) {
            if (distanceInches >= d[i] && distanceInches <= d[i + 1]) {
                double t = (distanceInches - d[i]) / (d[i + 1] - d[i]);
                return v[i] + t * (v[i + 1] - v[i]);
            }
        }
        return v[last];
    }

    private void updateShooter() {
        if (shooterTargetVel <= 0) {
            shooterL.setPower(0);
            shooterR.setPower(0);
            return;
        }
        tbhPowerL = tbh(shooterTargetVel, shooterL.getVelocity(), tbhPowerL);
        tbhPowerR = tbh(shooterTargetVel, shooterR.getVelocity(), tbhPowerR);
        shooterL.setPower(tbhPowerL);
        shooterR.setPower(tbhPowerR);
    }

    private static double tbh(double target, double current, double last) {
        if (current >= target - BioBuzzConstants.TBH_TOLERANCE) {
            return Math.max(BioBuzzConstants.TBH_MIN_POWER,
                    Math.min(1.0, last * BioBuzzConstants.TBH_HALF_FACTOR));
        }
        return BioBuzzConstants.TBH_FULL_POWER;
    }

    private void stopShooter() {
        shooterTargetVel = 0;
        tbhPowerL = tbhPowerR = 0;
        shooterL.setPower(0);
        shooterR.setPower(0);
    }

    private int avgShooterVel() {
        return (int) ((shooterL.getVelocity() + shooterR.getVelocity()) * 0.5);
    }

    private void updateAprilTag() {
        latestTag = null;
        for (AprilTagDetection det : aprilTag.getDetections()) {
            if (det.metadata != null && det.robotPose != null) {
                latestTag = det;
                break;
            }
        }
    }

    private void applyAprilTagToPinpoint() {
        if (latestTag == null) return;
        double x = latestTag.robotPose.getPosition().x;
        double y = latestTag.robotPose.getPosition().y;
        double yaw = latestTag.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, yaw));
    }

    private void handleIntakeIndex() {
        if (breakBeamFront != null) {
            boolean broken = isFrontBeamBroken();
            if (broken && !prevFrontBeamBroken) {
                setIndexPos(transfer.getCurrentPosition() + BioBuzzConstants.INDEX_STEP);
            }
            prevFrontBeamBroken = broken;
        }
        if (gamepad.right_trigger > BioBuzzConstants.TRIGGER_THRESHOLD) {
            intake.setPower(BioBuzzConstants.INTAKE_POWER);
            transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            transfer.setVelocity(BioBuzzConstants.PASSIVE_INDEX_VELOCITY);
        } else if (gamepad.left_trigger > BioBuzzConstants.TRIGGER_THRESHOLD) {
            intake.setPower(BioBuzzConstants.INTAKE_REVERSE_POWER);
        } else if (gamepad.dpadRightWasPressed()) {
            setIndexPos(transfer.getCurrentPosition() + BioBuzzConstants.INDEX_STEP);
            intake.setPower(1.0);
        } else if (gamepad.dpadLeftWasPressed()) {
            setIndexPos(transfer.getCurrentPosition() - BioBuzzConstants.INDEX_STEP);
            intake.setPower(-1.0);
        } else if (gamepad.circle) {
            transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            transfer.setPower(BioBuzzConstants.MAG_DUMP_POWER);
            intake.setPower(BioBuzzConstants.MAG_DUMP_POWER);
        } else {
            transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            transfer.setTargetPosition(transfer.getCurrentPosition());
            transfer.setPower(1.0);
            intake.setPower(0);
        }
    }

    private void setIndexPos(int pos) {
        transfer.setTargetPosition(pos);
        transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        transfer.setPower(1);
    }

    private boolean isFrontBeamBroken() {
        boolean raw = breakBeamFront.getState();
        return BioBuzzConstants.BREAK_BEAM_TRIPPED_WHEN_HIGH ? raw : !raw;
    }

    /** Field-centric mecanum — same as {@link Teleop_Basebotv2#fieldCentric}. */
    private void fieldCentric(double y, double x, double rx, double heading) {
        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);

        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
        rotX *= BioBuzzConstants.STRAFE_CORRECTION;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        fl.setPower((rotY + rotX + rx) / denominator);
        fr.setPower((rotY - rotX - rx) / denominator);
        bl.setPower((rotY - rotX + rx) / denominator);
        br.setPower((rotY + rotX - rx) / denominator);
    }

    private void stopDrive() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    private Servo getServo(String name) {
        return hardwareMap.getAllNames(Servo.class).contains(name) ? hardwareMap.get(Servo.class, name) : null;
    }

    private DigitalChannel getDigital(String name) {
        return hardwareMap.getAllNames(DigitalChannel.class).contains(name)
                ? hardwareMap.get(DigitalChannel.class, name) : null;
    }
}