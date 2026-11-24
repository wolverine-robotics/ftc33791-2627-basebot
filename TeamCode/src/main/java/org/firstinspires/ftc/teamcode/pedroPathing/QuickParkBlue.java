package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "QuickParkBlue", group = "Autonomous")
public class QuickParkBlue extends OpMode {

    private Follower follower;
    private int pathState;

    /** Red start/end rotated 180° about field center (72, 72). */
    private final Pose startPose = new Pose(104.8712, 135.6372, Math.toRadians(270));
    private final Pose endPose = new Pose(135.1667, 45.7763, Math.toRadians(270));

    private Path parkPath;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        parkPath = line(startPose, endPose).constant(Math.toRadians(270));
        follower.setPose(startPose);
    }

    @Override
    public void start() {
        pathState = 0;
    }

    @Override
    public void loop() {
        follower.update();

        switch (pathState) {
            case 0:
                follower.follow(parkPath);
                pathState = 1;
                break;
            case 1:
                break;
        }

        telemetry.addData("x", follower.pose().x());
        telemetry.addData("y", follower.pose().y());
        telemetry.addData("heading", follower.pose().heading());
        telemetry.addData("path completion %", follower.completion());
        telemetry.addData("busy", follower.isBusy());
        telemetry.update();
    }
}