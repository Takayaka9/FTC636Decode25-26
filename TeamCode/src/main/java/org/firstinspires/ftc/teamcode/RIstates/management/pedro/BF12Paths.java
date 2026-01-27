package org.firstinspires.ftc.teamcode.RIstates.management.pedro;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.RIstates.management.SystemManager;

public class BF12Paths {
    SystemManager manager;
    public BF12Paths(SystemManager manager) {
        this.manager = manager;
    }
    public PathChain fs0, pi1, i1, cs1, pi2, i2, cs2, pi3, i3, fs3, l;

    public void buildPaths() {

        fs0 = manager.follower.pathBuilder()
                .addPath(new BezierLine(manager.bluePoseLib.farStartPose, manager.bluePoseLib.farShootPose))
                .setLinearHeadingInterpolation(manager.bluePoseLib.farStartPose.getHeading(), manager.bluePoseLib.farShootPose.getHeading())
                .build();

        pi1 = manager.follower.pathBuilder().addPath(new BezierLine(manager.bluePoseLib.farShootPose, manager.bluePoseLib.pIntake1Pose))
                .setLinearHeadingInterpolation(manager.bluePoseLib.farShootPose.getHeading(), manager.bluePoseLib.pIntake1Pose.getHeading())
                .build();

        i1 = manager.follower.pathBuilder().addPath(new BezierLine(manager.bluePoseLib.pIntake1Pose, manager.bluePoseLib.intake1Pose))
                .setLinearHeadingInterpolation(manager.bluePoseLib.pIntake1Pose.getHeading(), manager.bluePoseLib.intake1Pose.getHeading())
                .build();

        cs1 = manager.follower.pathBuilder().addPath(new BezierLine(manager.bluePoseLib.intake1Pose, manager.bluePoseLib.nearShootPose))
                .setLinearHeadingInterpolation(manager.bluePoseLib.intake1Pose.getHeading(), manager.bluePoseLib.nearShootPose.getHeading())
                .build();

        pi2 = manager.follower.pathBuilder().addPath(new BezierLine(manager.bluePoseLib.nearShootPose, manager.bluePoseLib.pIntake2Pose))
                .setLinearHeadingInterpolation(manager.bluePoseLib.nearShootPose.getHeading(), manager.bluePoseLib.pIntake2Pose.getHeading())
                .build();

        i2 = manager.follower.pathBuilder().addPath(new BezierLine(manager.bluePoseLib.pIntake2Pose, manager.bluePoseLib.intake2Pose))
                .setLinearHeadingInterpolation(manager.bluePoseLib.pIntake2Pose.getHeading(), manager.bluePoseLib.intake2Pose.getHeading())
                .build();

        cs2 = manager.follower.pathBuilder().addPath(new BezierLine(manager.bluePoseLib.intake2Pose, manager.bluePoseLib.nearShootPose))
                .setLinearHeadingInterpolation(manager.bluePoseLib.intake2Pose.getHeading(), manager.bluePoseLib.nearShootPose.getHeading())
                .build();

        pi3 = manager.follower.pathBuilder().addPath(new BezierLine(manager.bluePoseLib.nearShootPose, manager.bluePoseLib.pIntake3Pose))
                .setLinearHeadingInterpolation(manager.bluePoseLib.nearShootPose.getHeading(), manager.bluePoseLib.pIntake3Pose.getHeading())
                .build();

        i3 = manager.follower.pathBuilder().addPath(new BezierLine(manager.bluePoseLib.pIntake3Pose, manager.bluePoseLib.intake3Pose))
                .setLinearHeadingInterpolation(manager.bluePoseLib.pIntake3Pose.getHeading(), manager.bluePoseLib.intake3Pose.getHeading())
                .build();

        fs3 = manager.follower.pathBuilder()
                .addPath(new BezierLine(manager.bluePoseLib.intake3Pose, manager.bluePoseLib.farShootPose))
                .setLinearHeadingInterpolation(manager.bluePoseLib.intake3Pose.getHeading(), manager.bluePoseLib.farShootPose.getHeading())
                .build();

        l = manager.follower.pathBuilder().addPath(new BezierLine(manager.bluePoseLib.farShootPose, manager.bluePoseLib.farLeavePose))
                .setLinearHeadingInterpolation(manager.bluePoseLib.farShootPose.getHeading(), manager.bluePoseLib.farLeavePose.getHeading())
                .build();
    }
}
