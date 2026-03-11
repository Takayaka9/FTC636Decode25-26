package org.firstinspires.ftc.teamcode.NewEnglands.pedro.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.NewEnglands.utils.pedroUtils.BasePathChain;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.pedroUtils.BuildPaths;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.alliance.Alliance;

public final class F12Paths extends BasePathChain implements BuildPaths {
    public F12Paths(Follower follower, Alliance alliance) {
        super(alliance, follower);
        buildPaths();
    }
    public PathChain fs0, pi1, i1, cs1, pi2, i2, cs2, pi3, i3, fs3, l, abort;

    @Override
    public void buildPaths() {

        fs0 = follower.pathBuilder()
                .addPath(new BezierLine(farStartPose, farShootPose))
                .setLinearHeadingInterpolation(farStartPose.getHeading(), farShootPose.getHeading())
                .build();

        pi1 = follower.pathBuilder().addPath(new BezierLine(farShootPose, pIntake1Pose))
                .setLinearHeadingInterpolation(farShootPose.getHeading(), pIntake1Pose.getHeading())
                .build();

        i1 = follower.pathBuilder().addPath(new BezierLine(pIntake1Pose, intake1Pose))
                .setLinearHeadingInterpolation(pIntake1Pose.getHeading(), intake1Pose.getHeading())
                .build();

        cs1 = follower.pathBuilder().addPath(new BezierLine(intake1Pose, nearShootPose))
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), nearShootPose.getHeading())
                .build();

        pi2 = follower.pathBuilder().addPath(new BezierLine(nearShootPose, pIntake2Pose))
                .setLinearHeadingInterpolation(nearShootPose.getHeading(), pIntake2Pose.getHeading())
                .build();

        i2 = follower.pathBuilder().addPath(new BezierLine(pIntake2Pose, intake2Pose))
                .setLinearHeadingInterpolation(pIntake2Pose.getHeading(), intake2Pose.getHeading())
                .build();

        cs2 = follower.pathBuilder().addPath(new BezierLine(intake2Pose, nearShootPose))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), nearShootPose.getHeading())
                .build();

        pi3 = follower.pathBuilder().addPath(new BezierLine(nearShootPose, pIntake3Pose))
                .setLinearHeadingInterpolation(nearShootPose.getHeading(), pIntake3Pose.getHeading())
                .build();

        i3 = follower.pathBuilder().addPath(new BezierLine(pIntake3Pose, intake3Pose))
                .setLinearHeadingInterpolation(pIntake3Pose.getHeading(), intake3Pose.getHeading())
                .build();

        fs3 = follower.pathBuilder()
                .addPath(new BezierLine(intake3Pose, farShootPose))
                .setLinearHeadingInterpolation(intake3Pose.getHeading(), farShootPose.getHeading())
                .build();

        l = follower.pathBuilder().addPath(new BezierLine(farShootPose, farLeavePose))
                .setLinearHeadingInterpolation(farShootPose.getHeading(), farLeavePose.getHeading())
                .build();
        abort = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), farLeavePose))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), farLeavePose.getHeading())
                .build();
    }
}
