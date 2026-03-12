package org.firstinspires.ftc.teamcode.NewEnglands.pedro.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.NewEnglands.utils.pedroUtils.BuildPaths;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.pedroUtils.BasePathChain;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.alliance.Alliance;

public final class C9Paths extends BasePathChain implements BuildPaths {
    public C9Paths(Follower follower, Alliance alliance) {
        super(alliance, follower);
        buildPaths();
    }

    public PathChain startToShoot, preIntakeSpike2, intakeSpike2, spike2ToShoot,
//            gateToShoot,
            preIntakeSpike1, intakeSpike1, spike1toShoot, preIntakeSpike3, intakeSpike3, spike3toShoot, shootToLeave, abort;

    public void buildPaths() {
        follower.pathBuilder().setGlobalTangentHeadingInterpolation();
        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(nearStartPose, nearShootPose))
                .setLinearHeadingInterpolation(nearStartPose.getHeading(), nearShootPose.getHeading())
                .build();
        preIntakeSpike2 = follower.pathBuilder()
                .addPath(new BezierLine(nearShootPose, pIntake2Pose))
                .setConstantHeadingInterpolation(intake2Pose.getHeading())
                .build();
        intakeSpike2 = follower.pathBuilder()
                .addPath(new BezierLine(pIntake2Pose, intake2Pose))
                .setConstantHeadingInterpolation(intake2Pose.getHeading())
                .build();
        spike2ToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(intake2Pose, pIntake2Pose, nearShootPose))
                .setConstantHeadingInterpolation(intake2Pose.getHeading())
                .build();
//        gateToShoot = follower.pathBuilder()
//                .addPath(new BezierCurve(emptyPose, pIntake2Pose, nearShootPose))
//                .setLinearHeadingInterpolation(emptyPose.getHeading(), nearShootPose.getHeading())
//                .build();
        preIntakeSpike1 = follower.pathBuilder()
                .addPath(new BezierLine(nearShootPose, pIntake1Pose))
                .setConstantHeadingInterpolation(intake1Pose.getHeading())
                .build();
        intakeSpike1 = follower.pathBuilder()
                .addPath(new BezierLine(pIntake1Pose, intake1Pose))
                .setConstantHeadingInterpolation(intake1Pose.getHeading())
                .build();
        spike1toShoot = follower.pathBuilder()
                .addPath(new BezierLine(intake1Pose, nearShootPose))
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), nearShootPose.getHeading())
                .build();
        preIntakeSpike3 = follower.pathBuilder()
                .addPath(new BezierLine(nearShootPose, pIntake3Pose))
                .setConstantHeadingInterpolation(intake3Pose.getHeading())
                .build();
        intakeSpike3 = follower.pathBuilder()
                .addPath(new BezierLine(pIntake3Pose, intake3Pose))
                .setConstantHeadingInterpolation(intake3Pose.getHeading())
                .build();
        spike3toShoot = follower.pathBuilder()
                .addPath(new BezierLine(intake3Pose, nearShootPose))
                .setLinearHeadingInterpolation(intake3Pose.getHeading(), nearShootPose.getHeading())
                .build();
        shootToLeave = follower.pathBuilder()
                .addPath(new BezierLine(nearShootPose, closeLeavePose))
                .build();
        abort = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), closeLeavePose))
                .build();
    }

}
