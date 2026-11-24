package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Path-only auto from {@code 2tipredside.pp.json} (Pedro Pathing editor export).
 */
@Autonomous(name = "2 Tip Red Side", group = "Autonomous")
public class TwoTipRedSide extends OpMode {

    private Follower follower;
    private boolean pathStarted;
    private Path route;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        route = TwoTipRoute.build(false);
        follower.setPose(TwoTipRoute.startPose(false));
    }

    @Override
    public void start() {
        pathStarted = false;
    }

    @Override
    public void loop() {
        follower.update();

        if (!pathStarted) {
            follower.follow(route);
            pathStarted = true;
        }

        telemetry.addData("x", follower.pose().x());
        telemetry.addData("y", follower.pose().y());
        telemetry.addData("heading (deg)", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("path completion %", follower.completion());
        telemetry.addData("busy", follower.isBusy());
        telemetry.addData("at end", follower.atParametricEnd());
        telemetry.update();
    }
}