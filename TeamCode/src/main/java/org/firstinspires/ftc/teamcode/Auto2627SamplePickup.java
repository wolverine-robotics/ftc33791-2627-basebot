package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

@Autonomous(name = "Auto2627 Sample Pickup", group = "Autonomous")
public class Auto2627SamplePickup extends LinearOpMode {

    private enum State { SEARCH, APPROACH, INTAKE, DONE }

    private DcMotorEx fl, fr, bl, br, intake;
    private GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        fl = hardwareMap.get(DcMotorEx.class, BioBuzzConstants.FL);
        fr = hardwareMap.get(DcMotorEx.class, BioBuzzConstants.FR);
        bl = hardwareMap.get(DcMotorEx.class, BioBuzzConstants.BL);
        br = hardwareMap.get(DcMotorEx.class, BioBuzzConstants.BR);
        intake = hardwareMap.get(DcMotorEx.class, BioBuzzConstants.INTAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, BioBuzzConstants.PINPOINT);
        TeleOp2627v1.configurePinpoint(pinpoint);
        limelight = hardwareMap.get(Limelight3A.class, BioBuzzConstants.LIMELIGHT);

        boolean nectar = false;
        while (!isStarted() && !isStopRequested()) {
            nectar = gamepad1.b;
            telemetry.addData("Pipeline", nectar ? "NECTAR (B)" : "POLLEN");
            telemetry.update();
        }

        int pipe = nectar ? BioBuzzConstants.LIMELIGHT_PIPELINE_NECTAR : BioBuzzConstants.LIMELIGHT_PIPELINE_POLLEN;
        limelight.pipelineSwitch(pipe);
        limelight.setPollRateHz(100);
        limelight.start();

        waitForStart();
        pinpoint.resetPosAndIMU();
        State state = State.SEARCH;
        ElapsedTime stateTimer = new ElapsedTime();
        ElapsedTime searchTimer = new ElapsedTime();

        while (opModeIsActive() && state != State.DONE) {
            pinpoint.update();

            double tx = 0, ty = 0;
            boolean hasTarget = false;
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.DetectorResult> dets = result.getDetectorResults();
                if (dets != null && !dets.isEmpty()
                        && dets.get(0).getTargetArea() >= BioBuzzConstants.LL_MIN_TARGET_AREA) {
                    hasTarget = true;
                    tx = result.getTx();
                    ty = result.getTy();
                }
            }

            switch (state) {
                case SEARCH:
                    driveRobotCentric(0, 0, 0.25);
                    if (hasTarget) {
                        stopDrive();
                        state = State.APPROACH;
                        stateTimer.reset();
                    } else if (searchTimer.seconds() > BioBuzzConstants.AUTO_SEARCH_TIMEOUT_SEC) {
                        stopDrive();
                        state = State.DONE;
                    }
                    break;
                case APPROACH:
                    if (!hasTarget) {
                        if (stateTimer.seconds() > 2) {
                            state = State.SEARCH;
                            searchTimer.reset();
                        }
                        stopDrive();
                        break;
                    }
                    double fwd = Range.clip(-ty * BioBuzzConstants.LL_DRIVE_KP_TY,
                            -BioBuzzConstants.AUTO_DRIVE_MAX_POWER, BioBuzzConstants.AUTO_DRIVE_MAX_POWER);
                    double turn = Range.clip(tx * BioBuzzConstants.LL_DRIVE_KP_TX,
                            -BioBuzzConstants.AUTO_DRIVE_MAX_POWER, BioBuzzConstants.AUTO_DRIVE_MAX_POWER);
                    driveRobotCentric(fwd, 0, turn);
                    if (Math.abs(tx) < BioBuzzConstants.LL_ALIGN_TX_TOL
                            && Math.abs(ty) < BioBuzzConstants.LL_ALIGN_TY_TOL) {
                        stopDrive();
                        state = State.INTAKE;
                        stateTimer.reset();
                    } else if (stateTimer.seconds() > BioBuzzConstants.AUTO_APPROACH_TIMEOUT_SEC) {
                        stopDrive();
                        state = State.SEARCH;
                        searchTimer.reset();
                    }
                    break;
                case INTAKE:
                    intake.setPower(BioBuzzConstants.INTAKE_POWER);
                    if (stateTimer.seconds() > BioBuzzConstants.AUTO_INTAKE_SEC) {
                        intake.setPower(0);
                        state = State.DONE;
                    }
                    break;
                default:
                    break;
            }

            telemetry.addData("State", state);
            telemetry.addData("Pose", "%.1f, %.1f",
                    pinpoint.getPosition().getX(DistanceUnit.INCH),
                    pinpoint.getPosition().getY(DistanceUnit.INCH));
            telemetry.addData("LL", hasTarget ? String.format("tx %.1f ty %.1f", tx, ty) : "none");
            telemetry.update();
        }

        stopDrive();
        limelight.stop();

        pinpoint.update();
        TeleOp2627v1.hasEndOfAutoPose = true;
        TeleOp2627v1.endOfAutoXIn = pinpoint.getPosition().getX(DistanceUnit.INCH);
        TeleOp2627v1.endOfAutoYIn = pinpoint.getPosition().getY(DistanceUnit.INCH);
        TeleOp2627v1.endOfAutoHeadingDeg = pinpoint.getHeading(AngleUnit.DEGREES);
    }

    private void driveRobotCentric(double forward, double strafe, double yaw) {
        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);
        strafe *= BioBuzzConstants.STRAFE_CORRECTION;
        double den = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(yaw), 1);
        fl.setPower((forward + strafe + yaw) / den);
        fr.setPower((forward - strafe - yaw) / den);
        bl.setPower((forward - strafe + yaw) / den);
        br.setPower((forward + strafe - yaw) / den);
    }

    private void stopDrive() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
}