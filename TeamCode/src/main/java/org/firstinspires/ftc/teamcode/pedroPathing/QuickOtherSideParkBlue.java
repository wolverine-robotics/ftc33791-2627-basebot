package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "QuickOtherParkBlue", group = "Autonomous")
public class QuickOtherSideParkBlue extends OpMode {

    private Follower follower;
    private int pathState;

    /** Red start/end rotated 180° about field center (72, 72). */
    private final Pose startPose = new Pose(106.30, 8.89, Math.toRadians(90));
    private final Pose endPose = new Pose(135.93, 26.19, Math.toRadians(90));

    private Path parkPath;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        parkPath = new Path(new BezierLine(startPose, endPose));
        parkPath.setConstantHeadingInterpolation(Math.toRadians(270));
        follower.setStartingPose(startPose);
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
                follower.followPath(parkPath);
                pathState = 1;
                break;
            case 1:
                break;
        }

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("path completion %", follower.getPathCompletion());
        telemetry.addData("busy", follower.isBusy());
        telemetry.update();
    }
}