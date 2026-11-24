package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

/**
 * Paths from {@code 2tipredside.pp.json}; blue alliance = 180° about field center (72, 72).
 */
final class TwoTipRoute {

    private static final double FIELD_SIZE_IN = 144.0;

    private TwoTipRoute() {}

    static Pose startPose(boolean blueAlliance) {
        return pt(55, 8, blueAlliance, Math.toRadians(90));
    }

    static Path build(boolean blueAlliance) {
        double h90 = headingDeg(90, blueAlliance);
        double h270 = headingDeg(270, blueAlliance);
        double h0 = blueAlliance ? Math.PI : 0;

        return path(
                line(pt(55, 8, blueAlliance), pt(55, 16, blueAlliance)).constant(h90),
                line(pt(55, 16, blueAlliance), pt(6, 16, blueAlliance)).constant(h90),
                line(pt(6, 16, blueAlliance), pt(6, 6, blueAlliance)).constant(h90),
                curve(
                        pt(6, 6, blueAlliance),
                        pt(23.383090733337333, 89.8936294397564, blueAlliance),
                        pt(47, 118, blueAlliance))
                        .heading(Interpolator.piecewise()
                                .until(0.2, Interpolator.tangent)
                                .until(1.0, Interpolator.tangent.reverse())),
                line(pt(47, 118, blueAlliance), pt(47, 134, blueAlliance)).constant(h270),
                line(pt(47, 134, blueAlliance), pt(47, 118, blueAlliance)).constant(h270),
                line(pt(47, 118, blueAlliance), pt(9.705445949847883, 111.1137848193829, blueAlliance))
                        .linear(h0, h0));
    }

    private static Pose pt(double x, double y, boolean blueAlliance) {
        if (blueAlliance) {
            return new Pose(FIELD_SIZE_IN - x, FIELD_SIZE_IN - y);
        }
        return new Pose(x, y);
    }

    private static Pose pt(double x, double y, boolean blueAlliance, double headingRad) {
        Pose p = pt(x, y, blueAlliance);
        if (blueAlliance) {
            return new Pose(p.x(), p.y(), headingRad + Math.PI);
        }
        return new Pose(p.x(), p.y(), headingRad);
    }

    private static double headingDeg(double degrees, boolean blueAlliance) {
        double d = blueAlliance ? degrees + 180.0 : degrees;
        return Math.toRadians(d);
    }
}