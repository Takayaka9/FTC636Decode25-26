package org.firstinspires.ftc.teamcode.afterPremier.util.pathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.afterPremier.util.Alliance;

public class ClosePaths {
    PoseLib p;
    Follower f;

    public ClosePaths(Alliance a, Follower f) {
        p = new PoseLib(a);
        this.f = f;
        buildPaths();
    }

    public PathChain startToShoot,
            intakeSpike2,
            spike2andEmpty,
            intakeSpike1,
            intakeSpike3,
            shootToGate,
            gateToShoot;

    public PathChain gateOpen,
            returnToShoot;

    public void buildPaths() {
        f.pathBuilder()
                .setGlobalDeceleration();

        startToShoot = f.pathBuilder()
                .addPath(new BezierLine(p.nearStartPose, p.nearShootPose))
                .setLinearHeadingInterpolation(p.nearStartPose.getHeading(), p.nearShootPose.getHeading(), 0.4)
                .build();

        intakeSpike1 = f.pathBuilder()
                .addPath(new BezierCurve(p.nearShootPose, p.intakeP1Pose, p.intake1Pose))
                .setConstantHeadingInterpolation(p.intake1Pose.getHeading())
                .addPath(new BezierLine(p.intake1Pose, p.nearShootPose))
                .setConstantHeadingInterpolation(p.nearShootPose.getHeading())
                .build();

        intakeSpike2 = f.pathBuilder()
                .addPath(new BezierCurve(p.nearShootPose, p.curvySpike2Control1Pose, p.curvySpike2Control2Pose, p.intake2Pose))
                .addPath(new BezierCurve(p.intake2Pose, p.curvySpike2Control1Pose, p.curvySpike2Control2Pose, p.nearShootPose))
                .setHeadingInterpolation(HeadingInterpolator.tangent.reverse())
                .build();

        spike2andEmpty = f.pathBuilder()
                .addPath(new BezierCurve(p.nearShootPose, p.curvySpike2Control1Pose, p.curvySpike2Control2Pose, p.intake2Pose))
                .setConstantHeadingInterpolation(p.nearShootPose.getHeading())
                .addParametricCallback(.995, () -> f.followPath(gateOpen))
                .build();

        returnToShoot = f.pathBuilder()
                .addPath(new BezierCurve(p.emptyPose, p.intake2Pose, p.intakeP2Pose, p.nearShootPose))
                .setLinearHeadingInterpolation(p.emptyPose.getHeading(), p.nearShootPose.getHeading())
                .build();

        gateOpen = f.pathBuilder()
                .addPath(new BezierCurve(p.intake2Pose, p.emptyPPose, p.emptyPose))
                .setLinearHeadingInterpolation(p.intake2Pose.getHeading(), p.emptyPose.getHeading())
//                .addPath(new BezierCurve(emptyPose, emptyPPose, intake2Pose))
//                .setLinearHeadingInterpolation(emptyPose.getHeading(), intake2Pose.getHeading())
                //.addParametricCallback(.995, () -> f.followPath(returnToShoot))
                .build();

        intakeSpike3 = f.pathBuilder()
                .addPath(new BezierCurve(p.nearShootPose, p.intakeP3Pose, p.intake3Pose))
                .setConstantHeadingInterpolation(p.nearShootPose.getHeading())
                .addPath(new BezierLine(p.intake3Pose, p.nearShootPose))
//                .addPath(new BezierCurve(intakeP3Pose, curvySpike3Control2Pose, curvySpike3Control1Pose, nearShootPose))
                .setConstantHeadingInterpolation(p.nearShootPose.getHeading())
                .build();

        shootToGate = f.pathBuilder()
                .addPath(new BezierCurve(p.nearShootPose, p.curvyGateControl1Pose, p.curvyGateControl2Pose, p.gatePose))
                .build();

        gateToShoot = f.pathBuilder()
                .addPath(new BezierCurve(p.gatePose, p.curvyGateControl2Pose, p.curvyGateControl1Pose, p.nearShootPose))
                .setConstantHeadingInterpolation(p.nearShootPose.getHeading())
                .build();
    }
}
