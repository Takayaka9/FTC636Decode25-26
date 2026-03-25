package org.firstinspires.ftc.teamcode.nePremier.pedro.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.nePremier.utils.pedroUtils.BasePathChain;
import org.firstinspires.ftc.teamcode.nePremier.utils.pedroUtils.BuildPaths;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.Alliance;

public final class F6Paths extends BasePathChain implements BuildPaths {

    public F6Paths(Follower follower, Alliance alliance) {
        super(alliance, follower);
        buildPaths();
    }

    public PathChain startToShoot, preIntakeSpike3, intakeSpike3, spike3toShoot, shootToLeave, abort;

    @Override
    public void buildPaths() {
        follower.pathBuilder().setGlobalTangentHeadingInterpolation();
        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(farStartPose, farShootPose))
                .setLinearHeadingInterpolation(farStartPose.getHeading(), farShootPose.getHeading())
                .build();
        preIntakeSpike3 = follower.pathBuilder()
                .addPath(new BezierLine(farShootPose, intakeP3Pose))
                .setConstantHeadingInterpolation(intake3Pose.getHeading())
                .build();
        intakeSpike3 = follower.pathBuilder()
                .addPath(new BezierLine(intakeP3Pose ,intake3Pose))
                .setConstantHeadingInterpolation(intake3Pose.getHeading())
                .build();
        spike3toShoot = follower.pathBuilder()
                .addPath(new BezierLine(intake3Pose, farShootPose))
                .setLinearHeadingInterpolation(intake3Pose.getHeading(), farShootPose.getHeading())
                .build();
    }



}