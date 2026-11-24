package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Bench rig: Limelight + Arducam only. D-pad L/R changes Limelight pipeline.
 */
@TeleOp(name = "ProgrammingRig2627", group = "Tuning")
public class ProgrammingRig2627 extends LinearOpMode {

    private Limelight3A limelight;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private int pipelineIndex;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, BioBuzzConstants.LIMELIGHT);
        pipelineIndex = 0;
        limelight.pipelineSwitch(pipelineIndex);
        limelight.setPollRateHz(100);
        limelight.start();

        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, BioBuzzConstants.WEBCAM))
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("D-pad L/R = Limelight pipeline");
        telemetry.addData(">", "Press START");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpadLeftWasPressed()) {
                pipelineIndex = Math.max(0, pipelineIndex - 1);
                limelight.pipelineSwitch(pipelineIndex);
            } else if (gamepad1.dpadRightWasPressed()) {
                pipelineIndex = Math.min(9, pipelineIndex + 1);
                limelight.pipelineSwitch(pipelineIndex);
            }

            LLResult result = limelight.getLatestResult();
            telemetry.addData("Pipeline", pipelineIndex);
            if (result != null && result.isValid()) {
                telemetry.addData("LL", "tx %.2f ty %.2f", result.getTx(), result.getTy());
            }

            List<AprilTagDetection> tags = aprilTag.getDetections();
            telemetry.addData("AprilTags", tags.size());
            telemetry.update();
            sleep(20);
        }

        limelight.stop();
        visionPortal.close();
    }
}