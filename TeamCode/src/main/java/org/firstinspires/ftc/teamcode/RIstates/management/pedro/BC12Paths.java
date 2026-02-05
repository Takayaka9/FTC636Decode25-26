package org.firstinspires.ftc.teamcode.RIstates.management.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.RIstates.management.pedro.utils.BluePoseLib;

public class BC12Paths extends BluePoseLib {
    Follower follower;
    public BC12Paths(Follower follower) {
        super();
        this.follower = follower;
    }

    public PathChain startToShoot, intakeSpike2, openGate, gateToShoot, intakeSpike1, spike1toShoot, intakeSpike3, spike3toShoot, shootToLeave, abort;

    public void buildPaths() {
        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(nearStartPose, nearShootPose))
                .setLinearHeadingInterpolation(nearStartPose.getHeading(), nearShootPose.getHeading())
                .build();
        intakeSpike2 = follower.pathBuilder()
                .addPath(new BezierCurve(nearShootPose, pIntake2Pose, intake2Pose))
                .setLinearHeadingInterpolation(nearShootPose.getHeading(), intake2Pose.getHeading())
                .setVelocityConstraint(.5)
                .build();
        openGate = follower.pathBuilder()
                .addPath(new BezierLine(intake2Pose, emptyPose))
                .setConstantHeadingInterpolation(intake2Pose.getHeading())
                .build();
        gateToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(emptyPose, pIntake2Pose, nearShootPose))
                .setLinearHeadingInterpolation(emptyPose.getHeading(), nearShootPose.getHeading())
                .build();
        intakeSpike1 = follower.pathBuilder()
                .addPath(new BezierCurve(nearShootPose, pIntake1Pose, intake1Pose))
                .setLinearHeadingInterpolation(nearShootPose.getHeading(), intake1Pose.getHeading())
                .setVelocityConstraint(.5)
                .build();
        spike1toShoot = follower.pathBuilder()
                .addPath(new BezierLine(intake1Pose, nearShootPose))
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), nearShootPose.getHeading())
                .build();
        intakeSpike3 = follower.pathBuilder()
                .addPath(new BezierCurve(nearShootPose, pIntake3Pose, intake3Pose))
                .setLinearHeadingInterpolation(nearShootPose.getHeading(), intake3Pose.getHeading())
                .setVelocityConstraint(.5)
                .build();
        spike3toShoot = follower.pathBuilder()
                .addPath(new BezierLine(intake3Pose, nearShootPose))
                .setLinearHeadingInterpolation(intake3Pose.getHeading(), nearShootPose.getHeading())
                .build();
        shootToLeave = follower.pathBuilder()
                .addPath(new BezierLine(nearShootPose, nearLeavePose))
                .setConstantHeadingInterpolation(nearShootPose.getHeading())
                .build();
        abort = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), nearLeavePose))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), nearLeavePose.getHeading())
                .build();
    }
}
