package org.firstinspires.ftc.teamcode.nePremier.pedro.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.nePremier.pedro.autoConstants.AutoConstants;
import org.firstinspires.ftc.teamcode.nePremier.utils.pedroUtils.BasePathChain;
import org.firstinspires.ftc.teamcode.nePremier.utils.pedroUtils.BuildPaths;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.Alliance;

public final class FPaths extends BasePathChain implements BuildPaths {
    public FPaths(Follower follower, Alliance alliance) {
        super(alliance, follower);
        buildPaths();
    }

    public PathChain startToShoot, preIntakeSpike3, intakeSpike3, spike3toShoot, shootToLeave, abort, shootToWall, wallToShoot, shootToFarIntake, farIntakeToShoot;

    @Override
    public void buildPaths() {
        follower.pathBuilder().setGlobalTangentHeadingInterpolation();
        follower.pathBuilder().setTValueConstraint(AutoConstants.globalTValue);
        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(farStartPose, farShootPose))
                .setLinearHeadingInterpolation(farStartPose.getHeading(), farShootPose.getHeading())
                .build();
        preIntakeSpike3 = follower.pathBuilder()
                .addPath(new BezierLine(farShootPose, intakeP3Pose))
                .setConstantHeadingInterpolation(intake3Pose.getHeading())
                .build();
        intakeSpike3 = follower.pathBuilder()
                .addPath(new BezierLine(farStartPose, intakeP3Pose))
                .setConstantHeadingInterpolation(intakeP3Pose.getHeading())
                .addPath(new BezierLine(intakeP3Pose ,intake3Pose))
                .setConstantHeadingInterpolation(intake3Pose.getHeading())
                .build();
        spike3toShoot = follower.pathBuilder()
                .addPath(new BezierLine(intake3Pose, farShootPose))
                .setConstantHeadingInterpolation(farShootPose.getHeading())
                .build();
        shootToWall = follower.pathBuilder()
                .addPath(new BezierLine(farShootPose, wallIntakePPose))
                .setLinearHeadingInterpolation(farShootPose.getHeading(), wallIntakePPose.getHeading())
                .addPath(new BezierLine(wallIntakePPose, wallIntakePose))
                .setConstantHeadingInterpolation(wallIntakePPose.getHeading())
                .build();
        wallToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(wallIntakePose, wallControlPose, farShootPose))
                .setLinearHeadingInterpolation(wallIntakePose.getHeading(), farShootPose.getHeading())
                .build();
        shootToFarIntake = follower.pathBuilder()
                .addPath(new BezierLine(farShootPose, farIntakePose))
                .setConstantHeadingInterpolation(farShootPose.getHeading())
                .build();
        farIntakeToShoot = follower.pathBuilder()
                .addPath(new BezierLine(farIntakePose, farShootPose))
                .setConstantHeadingInterpolation(farShootPose.getHeading())
                .build();

    }



}
