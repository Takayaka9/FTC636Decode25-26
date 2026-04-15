package org.firstinspires.ftc.teamcode.nePremier.pedro.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.nePremier.utils.pedroUtils.BasePathChain;
import org.firstinspires.ftc.teamcode.nePremier.utils.pedroUtils.BuildPaths;
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
            gateToShoot,
            returnToShoot;

    private PathChain gateOpen;


    @Override
    public void buildPaths() {
        follower.pathBuilder()
                .setGlobalDeceleration();

        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(nearStartPose, nearShootPose))
                .setLinearHeadingInterpolation(nearStartPose.getHeading(), nearShootPose.getHeading(), 0.4)
                .build();

        intakeSpike1 = follower.pathBuilder()
                .addPath(new BezierCurve(nearShootPose, intakeP1Pose, intake1Pose))
                .setConstantHeadingInterpolation(intake1Pose.getHeading())
                .addPath(new BezierLine(intake1Pose, nearShootPose))
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), nearShootPose.getHeading())
                .build();


        intakeSpike2 = follower.pathBuilder()
                .addPath(new BezierCurve(nearShootPose, curvySpike2Control1Pose, curvySpike2Control2Pose, intake2Pose))
                .setConstantHeadingInterpolation(nearShootPose.getHeading())
                .addPath(new BezierCurve(intake2Pose, curvySpike2Control1Pose, nearShootPose))
                .setConstantHeadingInterpolation(nearShootPose.getHeading())
                .build();

        spike2andEmpty = follower.pathBuilder()
                .addPath(new BezierCurve(nearShootPose, curvySpike2Control1Pose, curvySpike2Control2Pose, intake2Pose))
                .setConstantHeadingInterpolation(nearShootPose.getHeading())
//                .addParametricCallback(.95, () -> follower.followPath(gateOpen))
                .addPath(new BezierCurve(intake2Pose, intakeP2Pose, emptyPose))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), emptyPose.getHeading())
                .addPath(new BezierLine(emptyPose, nearShootPose))
                .setLinearHeadingInterpolation(emptyPose.getHeading(), nearShootPose.getHeading())
                .build();

        //not used
        gateOpen = follower.pathBuilder()
                .addPath(new BezierCurve(intake2Pose, emptyPPose, emptyPose))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), emptyPose.getHeading())
//                .addPath(new BezierCurve(emptyPose, emptyPPose, intake2Pose))
//                .setLinearHeadingInterpolation(emptyPose.getHeading(), intake2Pose.getHeading())
                //.addParametricCallback(.995, () -> follower.followPath(returnToShoot))
                .build();

        //not used
        returnToShoot = follower.pathBuilder()
                .addPath(new BezierLine(emptyPose, nearShootPose))
                .setLinearHeadingInterpolation(emptyPose.getHeading(), nearShootPose.getHeading())
                .build();

        intakeSpike3 = follower.pathBuilder()
                .addPath(new BezierCurve(nearShootPose, intakeP3Pose, intake3Pose))
                .setConstantHeadingInterpolation(nearShootPose.getHeading())
                .addPath(new BezierLine(intake3Pose, nearShootPose))
//                .addPath(new BezierCurve(intakeP3Pose, curvySpike3Control2Pose, curvySpike3Control1Pose, nearShootPose))
                .setConstantHeadingInterpolation(nearShootPose.getHeading())
                .build();

        shootToGate = follower.pathBuilder()
//                .addPath(new BezierCurve(nearShootPose, curvyGateControl1Pose, curvyGateControl2Pose, gatePose))
                .addPath(new BezierCurve(nearShootPose, gateControlPose, gatePose))
                .setLinearHeadingInterpolation(nearShootPose.getHeading(), gatePose.getHeading())
                .addPath(new BezierCurve(gatePose, curvyGateOpenControlPose, curvyGateOpenPose))
                .setConstantHeadingInterpolation(gatePose.getHeading())
                .build();

        //TODO: PLAN B FOR GATE INTAKE: switch gate timer to 0, do not use gateToShoot, and use this init instead
//        shootToGate = follower.pathBuilder()
//                .addPath(new BezierLine(nearShootPose, gatePose))
//                .setLinearHeadingInterpolation(nearShootPose.getHeading(), gatePose.getHeading())
//                .addPath(new BezierCurve(gatePose, curvyGateOpenControlPose, curvyGateOpenPose))
//                .setConstantHeadingInterpolation(gatePose.getHeading())
//                .addPath(new BezierLine(curvyGateOpenPose, nearShootPose))
//                .build();

        gateToShoot = follower.pathBuilder()
                .addPath(new BezierLine(curvyGateOpenPose, nearShootPose))
                .setLinearHeadingInterpolation(curvyGateOpenPose.getHeading(), nearShootPose.getHeading(), 0.5)
                .build();

    }

}
