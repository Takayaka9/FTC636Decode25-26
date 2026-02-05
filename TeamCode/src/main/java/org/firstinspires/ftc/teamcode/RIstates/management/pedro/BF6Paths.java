package org.firstinspires.ftc.teamcode.RIstates.management.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.RIstates.management.pedro.utils.BluePoseLib;
import org.firstinspires.ftc.teamcode.RIstates.management.pedro.utils.RedPoseLib;

public class BF6Paths extends BluePoseLib {
    Follower follower;

    public BF6Paths(Follower follower) {
        super();
        this.follower = follower;
    }

    public PathChain startToShoot, intakeSpike3, spike3toShoot, shootToLeave, abort;

    public void buildPaths() {
        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(farStartPose, farShootPose))
                .setLinearHeadingInterpolation(farStartPose.getHeading(), farShootPose.getHeading())
                .build();
        intakeSpike3 = follower.pathBuilder()
                .addPath(new BezierCurve(farShootPose, pIntake3Pose ,intake3Pose))
                .setLinearHeadingInterpolation(farShootPose.getHeading(), intake3Pose.getHeading())
                .setVelocityConstraint(.5)
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