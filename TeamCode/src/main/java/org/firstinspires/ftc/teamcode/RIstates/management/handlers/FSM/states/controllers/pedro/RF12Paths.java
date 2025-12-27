package org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.states.controllers.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.RIstates.management.SystemManager;

public class RF12Paths {
    SystemManager manager;
    public RF12Paths(SystemManager manager) {
        this.manager = manager;
    }
    private PathChain fs0, pi1, i1, cs1, pi2, i2, cs2, pi3, i3, fs3, l;

    public void buildPaths() {

        fs0 = manager.follower.pathBuilder()
                .addPath(new BezierLine(manager.poseLib.farStartPose, manager.poseLib.farShootPose))
                .setLinearHeadingInterpolation(manager.poseLib.farStartPose.getHeading(), manager.poseLib.farShootPose.getHeading())
                .build();

        pi1 = manager.follower.pathBuilder().addPath(new BezierLine(manager.poseLib.farShootPose, manager.poseLib.pIntake1Pose))
                .setLinearHeadingInterpolation(manager.poseLib.farShootPose.getHeading(), manager.poseLib.pIntake1Pose.getHeading())
                .build();

        i1 = manager.follower.pathBuilder().addPath(new BezierLine(manager.poseLib.pIntake1Pose, manager.poseLib.intake1Pose))
                .setLinearHeadingInterpolation(manager.poseLib.pIntake1Pose.getHeading(), manager.poseLib.intake1Pose.getHeading())
                .build();

        cs1 = manager.follower.pathBuilder().addPath(new BezierLine(manager.poseLib.intake1Pose, manager.poseLib.nearShootPose))
                .setLinearHeadingInterpolation(manager.poseLib.intake1Pose.getHeading(), manager.poseLib.nearShootPose.getHeading())
                .build();

        pi2 = manager.follower.pathBuilder().addPath(new BezierLine(manager.poseLib.nearShootPose, manager.poseLib.pIntake2Pose))
                .setLinearHeadingInterpolation(manager.poseLib.nearShootPose.getHeading(), manager.poseLib.pIntake2Pose.getHeading())
                .build();

        i2 = manager.follower.pathBuilder().addPath(new BezierLine(manager.poseLib.pIntake2Pose, manager.poseLib.intake2Pose))
                .setLinearHeadingInterpolation(manager.poseLib.pIntake2Pose.getHeading(), manager.poseLib.intake2Pose.getHeading())
                .build();

        cs2 = manager.follower.pathBuilder().addPath(new BezierLine(manager.poseLib.intake2Pose, manager.poseLib.nearShootPose))
                .setLinearHeadingInterpolation(manager.poseLib.intake2Pose.getHeading(), manager.poseLib.nearShootPose.getHeading())
                .build();

        pi3 = manager.follower.pathBuilder().addPath(new BezierLine(manager.poseLib.nearShootPose, manager.poseLib.pIntake3Pose))
                .setLinearHeadingInterpolation(manager.poseLib.nearShootPose.getHeading(), manager.poseLib.pIntake3Pose.getHeading())
                .build();

        i3 = manager.follower.pathBuilder().addPath(new BezierLine(manager.poseLib.pIntake3Pose, manager.poseLib.intake3Pose))
                .setLinearHeadingInterpolation(manager.poseLib.pIntake3Pose.getHeading(), manager.poseLib.intake3Pose.getHeading())
                .build();

        fs3 = manager.follower.pathBuilder()
                .addPath(new BezierLine(manager.poseLib.intake3Pose, manager.poseLib.farShootPose))
                .setLinearHeadingInterpolation(manager.poseLib.intake3Pose.getHeading(), manager.poseLib.farShootPose.getHeading())
                .build();

        //TODO: far leave pose needs to be done in pose lib
//        l = manager.follower.pathBuilder().addPath(new BezierLine(manager.poseLib.nearShootPose, leave))
//                .setLinearHeadingInterpolation(manager.poseLib.nearShootPose.getHeading(), leave.getHeading())
//                .build();
    }
}
