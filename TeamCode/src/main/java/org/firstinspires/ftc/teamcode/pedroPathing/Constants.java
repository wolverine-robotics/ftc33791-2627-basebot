package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    // Re-tune via AutoTune (http://192.168.43.1:10158) and paste generated configs here.
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("fl");
        c.frontRightName.set("fr");
        c.backLeftName.set("bl");
        c.backRightName.set("br");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    // Former v2 Pinpoint tuning (107.31371 mm forward pod Y ≈ 4.2244 in).
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(0.0);
        c.yPodOffset.set(4.2244);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    // Seed values from legacy FollowerConstants; run Foresight AutoTune for production use.
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        c.maxAchievableForwardVelocity.set(81.28767);
        c.maxAchievableStrafeVelocity.set(59.1127);
        c.naturalForwardDeceleration.set(29.3404);
        c.naturalStrafeDeceleration.set(77.7388);
    });

    public static Follower createFollower(HardwareMap hardwareMap) {
        Localizer localizer = new PinpointLocalizer(hardwareMap, localizerConfig);
        Drivetrain drivetrain = new Mecanum(hardwareMap, drivetrainConfig);
        return new Follower(localizer, drivetrain, new Foresight(foresightConfig));
    }
}