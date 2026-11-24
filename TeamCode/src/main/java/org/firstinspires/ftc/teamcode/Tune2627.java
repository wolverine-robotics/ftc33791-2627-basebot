package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Field tuner for shooter / hood / turret / feeder (Pinpoint + hive targets assumed already good).
 * Gamepad1:
 *   Drive — sticks (field-centric, same as teleop)
 *   Touchpad — red / blue hive
 *   Cross — freeze drive + update aim
 *   RB (hold) — aim + flywheel RPM from distance table + {@link #shooterPlusOffset}
 *   LB — stop flywheel
 *   RT — add 50 ticks/s manual trim to shooter target
 *   D-pad Up/Down — manual hood servo position ±0.02 (logs for BioBuzzConstants)
 *   D-pad Left/Right — {@link #shooterPlusOffset} ±25
 *   A/B — {@link #turretKpLive} ±0.005
 *   Y — log suggested table row: current distance + current flywheel avg ticks/s
 */
@TeleOp(name = "Tune2627", group = "Tuning")
public class Tune2627 extends LinearOpMode {

    private GoBildaPinpointDriver pinpoint;
    private DcMotorEx fl, fr, bl, br, shooterL, shooterR;
    private Servo hood1, hood2;
    private final List<CRServo> turretMotors = new ArrayList<>();

    private boolean redAlliance = true;
    private double shooterTargetVel;
    private double tbhPowerL, tbhPowerR;
    private double shooterPlusOffset = BioBuzzConstants.SHOOTER_PLUS;
    private double turretKpLive = BioBuzzConstants.TURRET_KP;
    private double manualHoodServo = 0.35;
    private boolean useManualHood = false;

    @Override
    public void runOpMode() throws InterruptedException {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, BioBuzzConstants.PINPOINT);
        TeleOp2627v1.configurePinpoint(pinpoint);

        fl = hardwareMap.get(DcMotorEx.class, BioBuzzConstants.FL);
        fr = hardwareMap.get(DcMotorEx.class, BioBuzzConstants.FR);
        bl = hardwareMap.get(DcMotorEx.class, BioBuzzConstants.BL);
        br = hardwareMap.get(DcMotorEx.class, BioBuzzConstants.BR);
        shooterL = hardwareMap.get(DcMotorEx.class, BioBuzzConstants.SHOOTER_L);
        shooterR = hardwareMap.get(DcMotorEx.class, BioBuzzConstants.SHOOTER_R);
        shooterL.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterR.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hood1 = getServo(BioBuzzConstants.HOOD1);
        hood2 = getServo(BioBuzzConstants.HOOD2);
        for (String name : new String[]{
                BioBuzzConstants.TURRET1, BioBuzzConstants.TURRET2,
                BioBuzzConstants.TURRET3, BioBuzzConstants.TURRET4}) {
            if (hardwareMap.getAllNames(CRServo.class).contains(name)) {
                turretMotors.add(hardwareMap.get(CRServo.class, name));
            }
        }

        telemetry.addLine("Tune2627: hood / shooter / turret / table (not Pinpoint or hive XY)");
        telemetry.update();
        waitForStart();

        if (TeleOp2627v1.hasEndOfAutoPose) {
            pinpoint.setPosition(new org.firstinspires.ftc.robotcore.external.navigation.Pose2D(
                    DistanceUnit.INCH, TeleOp2627v1.endOfAutoXIn, TeleOp2627v1.endOfAutoYIn,
                    AngleUnit.DEGREES, TeleOp2627v1.endOfAutoHeadingDeg));
        }

        while (opModeIsActive()) {
            pinpoint.update();

            if (gamepad1.touchpadWasPressed()) {
                redAlliance = !redAlliance;
            }
            if (gamepad1.dpadUpWasPressed()) {
                useManualHood = true;
                manualHoodServo = Range.clip(manualHoodServo + 0.02, 0, 1);
            }
            if (gamepad1.dpadDownWasPressed()) {
                useManualHood = true;
                manualHoodServo = Range.clip(manualHoodServo - 0.02, 0, 1);
            }
            if (gamepad1.dpadLeftWasPressed()) {
                shooterPlusOffset -= 25;
            }
            if (gamepad1.dpadRightWasPressed()) {
                shooterPlusOffset += 25;
            }
            if (gamepad1.aWasPressed()) {
                turretKpLive = Math.max(0.005, turretKpLive - 0.005);
            }
            if (gamepad1.bWasPressed()) {
                turretKpLive = Math.min(0.15, turretKpLive + 0.005);
            }
            if (gamepad1.yWasPressed()) {
                double dist = hiveDistanceIn();
                int vel = avgShooterVel();
                telemetry.addLine(String.format(Locale.US, "TABLE HINT: dist %.2f -> vel %d", dist, vel));
            }

            double heading = pinpoint.getHeading(AngleUnit.RADIANS);
            double dist = hiveDistanceIn();
            double hoodDeg = hoodAngleForDistance(dist);
            double turretErr = turretErrorDeg(redAlliance);

            boolean aim = gamepad1.cross || gamepad1.right_bumper;
            if (aim) {
                if (useManualHood) {
                    setHoodServo(manualHoodServo);
                } else {
                    setHoodFromModel(hoodDeg);
                }
                driveTurret(turretErr);
            } else {
                stopTurret();
            }

            if (gamepad1.right_bumper) {
                shooterTargetVel = predictShooterVelocity(dist) + shooterPlusOffset;
                if (gamepad1.right_trigger > 0.2) {
                    shooterTargetVel += 50;
                }
            }
            if (gamepad1.left_bumper) {
                stopShooter();
            }
            updateShooter();

            if (!gamepad1.cross) {
                fieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x,
                        gamepad1.right_stick_x * BioBuzzConstants.PIVOT_MULTIPLIER, heading);
            } else {
                stopDrive();
            }

            telemetry.addData("Hive dist (in)", "%.2f", dist);
            telemetry.addData("Turret err (deg)", "%.2f", turretErr);
            telemetry.addData("Hood model (deg)", "%.2f", hoodDeg);
            telemetry.addData("Hood servo", useManualHood ? "MANUAL" : "MODEL");
            telemetry.addData("manualHoodServo", "%.3f  -> copy to HOOD_SERVO_MIN/MAX", manualHoodServo);
            telemetry.addData("Predicted shooter", "%.0f + plus %d = %.0f",
                    predictShooterVelocity(dist), (int) shooterPlusOffset,
                    predictShooterVelocity(dist) + shooterPlusOffset);
            telemetry.addData("Shooter", "tgt %d avg %d", (int) shooterTargetVel, avgShooterVel());
            telemetry.addData("Copy to constants", "SHOOTER_PLUS=%d TURRET_KP=%.3f",
                    (int) shooterPlusOffset, turretKpLive);
            telemetry.addData("Hood constants check", "NEAR %.0f@%.0fin FAR %.0f@%.0fin",
                    BioBuzzConstants.HOOD_ANGLE_NEAR_DEG, BioBuzzConstants.HOOD_DISTANCE_NEAR_IN,
                    BioBuzzConstants.HOOD_ANGLE_FAR_DEG, BioBuzzConstants.HOOD_DISTANCE_FAR_IN);
            telemetry.update();
        }

        stopShooter();
        stopTurret();
    }

    private double hiveDistanceIn() {
        double rx = pinpoint.getPosition().getX(DistanceUnit.INCH);
        double ry = pinpoint.getPosition().getY(DistanceUnit.INCH);
        double hx = redAlliance ? BioBuzzConstants.HIVE_RED_X : BioBuzzConstants.HIVE_BLUE_X;
        double hy = redAlliance ? BioBuzzConstants.HIVE_RED_Y : BioBuzzConstants.HIVE_BLUE_Y;
        return Math.hypot(hx - rx, hy - ry);
    }

    private double turretErrorDeg(boolean red) {
        double rx = pinpoint.getPosition().getX(DistanceUnit.INCH);
        double ry = pinpoint.getPosition().getY(DistanceUnit.INCH);
        double hx = red ? BioBuzzConstants.HIVE_RED_X : BioBuzzConstants.HIVE_BLUE_X;
        double hy = red ? BioBuzzConstants.HIVE_RED_Y : BioBuzzConstants.HIVE_BLUE_Y;
        double bearing = Math.toDegrees(Math.atan2(hy - ry, hx - rx));
        return AngleUnit.normalizeDegrees(bearing - pinpoint.getHeading(AngleUnit.DEGREES));
    }

    private static double hoodAngleForDistance(double distanceInches) {
        double t = Range.clip((distanceInches - BioBuzzConstants.HOOD_DISTANCE_NEAR_IN)
                / (BioBuzzConstants.HOOD_DISTANCE_FAR_IN - BioBuzzConstants.HOOD_DISTANCE_NEAR_IN), 0, 1);
        return BioBuzzConstants.HOOD_ANGLE_NEAR_DEG
                + t * (BioBuzzConstants.HOOD_ANGLE_FAR_DEG - BioBuzzConstants.HOOD_ANGLE_NEAR_DEG);
    }

    private void setHoodFromModel(double hoodDeg) {
        if (hood1 == null || hood2 == null) return;
        double span = BioBuzzConstants.HOOD_ANGLE_FAR_DEG - BioBuzzConstants.HOOD_ANGLE_NEAR_DEG;
        double u = Range.clip((hoodDeg - BioBuzzConstants.HOOD_ANGLE_NEAR_DEG) / span, 0, 1);
        double pos = BioBuzzConstants.HOOD_SERVO_MIN
                + u * (BioBuzzConstants.HOOD_SERVO_MAX - BioBuzzConstants.HOOD_SERVO_MIN);
        hood1.setPosition(pos);
        hood2.setPosition(pos);
    }

    private void setHoodServo(double pos) {
        if (hood1 != null) hood1.setPosition(pos);
        if (hood2 != null) hood2.setPosition(pos);
    }

    private void driveTurret(double turretErrorDeg) {
        if (turretMotors.isEmpty()) return;
        double power = Range.clip(turretErrorDeg * turretKpLive,
                -BioBuzzConstants.TURRET_MAX_POWER, BioBuzzConstants.TURRET_MAX_POWER);
        if (Math.abs(turretErrorDeg) < 1.5) power = 0;
        for (CRServo t : turretMotors) t.setPower(power);
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

    private void fieldCentric(double y, double x, double rx, double heading) {
        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);
        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
        rotX *= BioBuzzConstants.STRAFE_CORRECTION;
        double den = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        fl.setPower((rotY + rotX + rx) / den);
        fr.setPower((rotY - rotX - rx) / den);
        bl.setPower((rotY - rotX + rx) / den);
        br.setPower((rotY + rotX - rx) / den);
    }

    private void stopDrive() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    private Servo getServo(String name) {
        return hardwareMap.getAllNames(Servo.class).contains(name)
                ? hardwareMap.get(Servo.class, name) : null;
    }
}