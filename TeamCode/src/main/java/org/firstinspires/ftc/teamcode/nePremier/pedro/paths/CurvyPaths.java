package org.firstinspires.ftc.teamcode.nePremier.pedro.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.nePremier.pedro.autoConstants.AutoConstants;
import org.firstinspires.ftc.teamcode.nePremier.utils.pedroUtils.BuildPaths;
import org.firstinspires.ftc.teamcode.nePremier.utils.pedroUtils.BasePathChain;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.Alliance;

public final class CurvyPaths extends BasePathChain implements BuildPaths {
    public CurvyPaths(Follower follower, Alliance alliance) {
        super(alliance, follower);
        buildPaths();
    }

    public PathChain startToShoot,
            intakeSpike2,
            spike2andEmpty,
            intakeSpike1,
            intakeSpike3,
            shootToGate,
            gateToShoot;

    private PathChain gateOpen,
            returnToShoot;

    @Override
    public void buildPaths() {
        follower.pathBuilder().setGlobalTangentHeadingInterpolation()
                .setGlobalDeceleration();

        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(nearStartPose, nearShootPose))
                .setLinearHeadingInterpolation(nearStartPose.getHeading(), nearShootPose.getHeading(), 0.4)
                .build();

        intakeSpike1 = follower.pathBuilder()
                .addPath(new BezierCurve(nearShootPose, intakeP1Pose, intake1Pose))
                .setConstantHeadingInterpolation(nearShootPose.getHeading())
                .addPath(new BezierCurve(intake1Pose, nearShootPose))
                .setConstantHeadingInterpolation(nearShootPose.getHeading())
                .build();


        intakeSpike2 = follower.pathBuilder()
                .addPath(new BezierCurve(nearShootPose, new Pose(113,78), new Pose(82,58), intake2Pose))
                .addPath(new BezierCurve(intake2Pose, new Pose(113,78), new Pose(82,58), nearShootPose))
                .setHeadingInterpolation(HeadingInterpolator.tangent.reverse())
                .build();

        spike2andEmpty = follower.pathBuilder()
                .addPath(new BezierCurve(nearShootPose, new Pose(113,78), new Pose(82,58), intake2Pose))
                .addParametricCallback(.995, () -> follower.followPath(gateOpen))
                .build();
        returnToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(intake2Pose, new Pose(113,78), new Pose(82,58), nearShootPose))
                .setHeadingInterpolation(HeadingInterpolator.tangent.reverse())
                .build();
        gateOpen = follower.pathBuilder()
                .addPath(new BezierCurve(intake2Pose, emptyPPose, emptyPose))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), emptyPose.getHeading())
                .addPath(new BezierCurve(emptyPose, emptyPPose, intake2Pose))
                .setLinearHeadingInterpolation(emptyPose.getHeading(), intake2Pose.getHeading())
                .addParametricCallback(.995, () -> follower.followPath(returnToShoot))
                .build();

        intakeSpike3 = follower.pathBuilder()
                .addPath(new BezierCurve(nearShootPose, new Pose(113,78), new Pose(82,28), intakeP3Pose))
                .addPath(new BezierCurve(intakeP3Pose, new Pose(82,28), new Pose(113,78), nearShootPose))
                .setHeadingInterpolation(HeadingInterpolator.tangent.reverse())
                .build();

        shootToGate = follower.pathBuilder()
                .addPath(new BezierCurve(nearShootPose, new Pose(125,82), new Pose(105,56), gatePose))
                .build();

        gateToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(gatePose, new Pose(105,56), new Pose(125,82), nearShootPose))
                .build();

    }

}
