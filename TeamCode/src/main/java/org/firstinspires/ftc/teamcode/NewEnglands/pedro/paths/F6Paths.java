package org.firstinspires.ftc.teamcode.NewEnglands.pedro.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.NewEnglands.utils.pedroUtils.BasePathChain;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.pedroUtils.BuildPaths;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.alliance.Alliance;

public final class F6Paths extends BasePathChain implements BuildPaths {

    public F6Paths(Follower follower, Alliance alliance) {
        super(alliance, follower);
        buildPaths();
    }

    public PathChain startToShoot, preIntakeSpike3, intakeSpike3, spike3toShoot, shootToLeave, abort;

    public void buildPaths() {
        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(farStartPose, farShootPose))
                .setLinearHeadingInterpolation(farStartPose.getHeading(), farShootPose.getHeading())
                .build();
        preIntakeSpike3 = follower.pathBuilder()
                .addPath(new BezierLine(farShootPose, pIntake3Pose))
                .setConstantHeadingInterpolation(intake3Pose.getHeading())
                .build();
        intakeSpike3 = follower.pathBuilder()
                .addPath(new BezierLine(pIntake3Pose ,intake3Pose))
                .setConstantHeadingInterpolation(intake3Pose.getHeading())
                .build();
        spike3toShoot = follower.pathBuilder()
                .addPath(new BezierLine(intake3Pose, farShootPose))
                .setLinearHeadingInterpolation(intake3Pose.getHeading(), farShootPose.getHeading())
                .build();
        shootToLeave = follower.pathBuilder()
                .addPath(new BezierLine(farShootPose, farLeavePose))
                .setConstantHeadingInterpolation(farShootPose.getHeading())
                .build();
        abort = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), farLeavePose))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), farLeavePose.getHeading())
                .build();
    }



}