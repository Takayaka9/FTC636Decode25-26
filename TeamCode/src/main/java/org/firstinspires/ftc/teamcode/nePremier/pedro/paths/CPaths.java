package org.firstinspires.ftc.teamcode.nePremier.pedro.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.nePremier.pedro.autoConstants.AutoConstants;
import org.firstinspires.ftc.teamcode.nePremier.utils.pedroUtils.BuildPaths;
import org.firstinspires.ftc.teamcode.nePremier.utils.pedroUtils.BasePathChain;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.Alliance;

public final class CPaths extends BasePathChain implements BuildPaths {
    public CPaths(Follower follower, Alliance alliance) {
        super(alliance, follower);
        buildPaths();
    }

    public PathChain startToShoot;
    public PathChain preIntakeSpike2, intakeSpike2, spike2ToShoot, spike2andEmpty, emptyToShoot;
    public PathChain preIntakeSpike1, intakeSpike1, spike1toShoot;
    public PathChain preIntakeSpike3, intakeSpike3, spike3toShoot;
    public PathChain shootToGate, gateToShoot;

    @Override
    public void buildPaths() {
        follower.pathBuilder().setGlobalTangentHeadingInterpolation();
        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(nearStartPose, nearShootPose))
                .setLinearHeadingInterpolation(nearStartPose.getHeading(), nearShootPose.getHeading())
                .build();
        preIntakeSpike2 = follower.pathBuilder()
                .addPath(new BezierLine(nearShootPose, intakeP2Pose))
                .setConstantHeadingInterpolation(intake2Pose.getHeading())
                .build();
        intakeSpike2 = follower.pathBuilder()
                .addPath(new BezierLine(nearShootPose, intakeP2Pose))
                .setConstantHeadingInterpolation(intake2Pose.getHeading())
                .addPath(new BezierLine(intakeP2Pose, intake2Pose))
                .setConstantHeadingInterpolation(intake2Pose.getHeading())
                .build();
        spike2ToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(intake2Pose, intakeP2Pose, nearShootPose))
                .setConstantHeadingInterpolation(intake2Pose.getHeading())
                .build();
        preIntakeSpike1 = follower.pathBuilder()
                .addPath(new BezierLine(nearShootPose, intakeP1Pose))
                .setConstantHeadingInterpolation(intake1Pose.getHeading())
                .build();
        intakeSpike1 = follower.pathBuilder()
                .addPath(new BezierLine(nearShootPose, intakeP1Pose))
                .setConstantHeadingInterpolation(intake1Pose.getHeading())
                .addPath(new BezierLine(intakeP1Pose, intake1Pose))
                .setConstantHeadingInterpolation(intake1Pose.getHeading())
                .build();
        spike1toShoot = follower.pathBuilder()
                .addPath(new BezierLine(intake1Pose, finalShootPose))
                .setConstantHeadingInterpolation(finalShootPose.getHeading())
                .build();
        preIntakeSpike3 = follower.pathBuilder()
                .addPath(new BezierLine(nearShootPose, intakeP3Pose))
                .setConstantHeadingInterpolation(intake3Pose.getHeading())
                .build();
        intakeSpike3 = follower.pathBuilder()
                .addPath(new BezierLine(nearShootPose, intakeP3Pose))
                .setConstantHeadingInterpolation(intake3Pose.getHeading())
                .addPath(new BezierLine(intakeP3Pose, intake3Pose))
                .setConstantHeadingInterpolation(intake3Pose.getHeading())
                .build();
        spike3toShoot = follower.pathBuilder()
                .addPath(new BezierLine(intake3Pose, nearShootPose))
                .setLinearHeadingInterpolation(intake3Pose.getHeading(), nearShootPose.getHeading())
                .build();
        spike2andEmpty = follower.pathBuilder()
                .addPath(new BezierLine(intake2Pose, emptyPPose))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), emptyPPose.getHeading())
                .addPath(new BezierLine(emptyPPose, emptyPose))
                .setConstantHeadingInterpolation(emptyPPose.getHeading())
                .build();
        emptyToShoot = follower.pathBuilder()
                .addPath(new BezierLine(emptyPose, nearShootPose))
                .setLinearHeadingInterpolation(emptyPose.getHeading(), nearShootPose.getHeading())
                .build();
        shootToGate = follower.pathBuilder()
                .addPath(new BezierLine(nearShootPose, gatePose))
                .setTangentHeadingInterpolation()
                .build();
        gateToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(gatePose, new Pose(96, 61, Math.toRadians(0)), nearShootPose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), nearShootPose.getHeading())
                .build();
    }

}
