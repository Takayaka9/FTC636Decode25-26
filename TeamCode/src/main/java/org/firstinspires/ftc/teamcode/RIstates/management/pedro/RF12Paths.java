package org.firstinspires.ftc.teamcode.RIstates.management.pedro;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.RIstates.management.SystemManager;

public class RF12Paths {
    SystemManager manager;
    public RF12Paths(SystemManager manager) {
        this.manager = manager;
    }
    public PathChain fs0, pi1, i1, cs1, pi2, i2, cs2, pi3, i3, fs3, l;

    public void buildPaths() {

        fs0 = manager.follower.pathBuilder()
                .addPath(new BezierLine(manager.redPoseLib.farStartPose, manager.redPoseLib.farShootPose))
                .setLinearHeadingInterpolation(manager.redPoseLib.farStartPose.getHeading(), manager.redPoseLib.farShootPose.getHeading())
                .build();

        pi1 = manager.follower.pathBuilder().addPath(new BezierLine(manager.redPoseLib.farShootPose, manager.redPoseLib.pIntake1Pose))
                .setLinearHeadingInterpolation(manager.redPoseLib.farShootPose.getHeading(), manager.redPoseLib.pIntake1Pose.getHeading())
                .build();

        i1 = manager.follower.pathBuilder().addPath(new BezierLine(manager.redPoseLib.pIntake1Pose, manager.redPoseLib.intake1Pose))
                .setLinearHeadingInterpolation(manager.redPoseLib.pIntake1Pose.getHeading(), manager.redPoseLib.intake1Pose.getHeading())
                .build();

        cs1 = manager.follower.pathBuilder().addPath(new BezierLine(manager.redPoseLib.intake1Pose, manager.redPoseLib.nearShootPose))
                .setLinearHeadingInterpolation(manager.redPoseLib.intake1Pose.getHeading(), manager.redPoseLib.nearShootPose.getHeading())
                .build();

        pi2 = manager.follower.pathBuilder().addPath(new BezierLine(manager.redPoseLib.nearShootPose, manager.redPoseLib.pIntake2Pose))
                .setLinearHeadingInterpolation(manager.redPoseLib.nearShootPose.getHeading(), manager.redPoseLib.pIntake2Pose.getHeading())
                .build();

        i2 = manager.follower.pathBuilder().addPath(new BezierLine(manager.redPoseLib.pIntake2Pose, manager.redPoseLib.intake2Pose))
                .setLinearHeadingInterpolation(manager.redPoseLib.pIntake2Pose.getHeading(), manager.redPoseLib.intake2Pose.getHeading())
                .build();

        cs2 = manager.follower.pathBuilder().addPath(new BezierLine(manager.redPoseLib.intake2Pose, manager.redPoseLib.nearShootPose))
                .setLinearHeadingInterpolation(manager.redPoseLib.intake2Pose.getHeading(), manager.redPoseLib.nearShootPose.getHeading())
                .build();

        pi3 = manager.follower.pathBuilder().addPath(new BezierLine(manager.redPoseLib.nearShootPose, manager.redPoseLib.pIntake3Pose))
                .setLinearHeadingInterpolation(manager.redPoseLib.nearShootPose.getHeading(), manager.redPoseLib.pIntake3Pose.getHeading())
                .build();

        i3 = manager.follower.pathBuilder().addPath(new BezierLine(manager.redPoseLib.pIntake3Pose, manager.redPoseLib.intake3Pose))
                .setLinearHeadingInterpolation(manager.redPoseLib.pIntake3Pose.getHeading(), manager.redPoseLib.intake3Pose.getHeading())
                .build();

        fs3 = manager.follower.pathBuilder()
                .addPath(new BezierLine(manager.redPoseLib.intake3Pose, manager.redPoseLib.farShootPose))
                .setLinearHeadingInterpolation(manager.redPoseLib.intake3Pose.getHeading(), manager.redPoseLib.farShootPose.getHeading())
                .build();

        l = manager.follower.pathBuilder().addPath(new BezierLine(manager.redPoseLib.farShootPose, manager.redPoseLib.farLeavePose))
                .setLinearHeadingInterpolation(manager.redPoseLib.farShootPose.getHeading(), manager.redPoseLib.farLeavePose.getHeading())
                .build();
    }
}
