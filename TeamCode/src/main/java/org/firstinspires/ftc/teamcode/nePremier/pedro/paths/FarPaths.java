package org.firstinspires.ftc.teamcode.nePremier.pedro.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.nePremier.utils.pedroUtils.BasePathChain;
import org.firstinspires.ftc.teamcode.nePremier.utils.pedroUtils.BuildPaths;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.Alliance;

public final class FarPaths extends BasePathChain implements BuildPaths {
    public FarPaths(Follower follower, Alliance alliance) {
        super(alliance, follower);
        buildPaths();
    }

    public PathChain startToShoot,
            intakeSpike3,
            spike3toShoot,
            shootToWall,
            wallToShoot,
            shootToFarIntake,
            farIntakeToShoot;

    @Override
    public void buildPaths() {
        follower.pathBuilder().setGlobalTangentHeadingInterpolation()
                .setGlobalDeceleration();
        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(farStartPose, farShootPose))
                .setLinearHeadingInterpolation(farStartPose.getHeading(), farShootPose.getHeading())
                .build();
        intakeSpike3 = follower.pathBuilder()
                .addPath(new BezierCurve(farShootPose, intakeP3Pose, intake3Pose))
                .setLinearHeadingInterpolation(farShootPose.getHeading(), intakeP3Pose.getHeading(), 0.6)
                .build();
        spike3toShoot = follower.pathBuilder()
                .addPath(new BezierLine(intake3Pose, farShootPose))
                .setConstantHeadingInterpolation(farShootPose.getHeading())
                .build();
        shootToWall = follower.pathBuilder()
                .addPath(new BezierLine(farShootPose, wallIntakePPose))
                .setLinearHeadingInterpolation(farShootPose.getHeading(), wallIntakePose.getHeading())
                .addPath(new BezierLine(wallIntakePPose, wallIntakePose))
                .setConstantHeadingInterpolation(wallIntakePose.getHeading())
                .build();
        wallToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(wallIntakePose, wallControlPose, farShootPose))
                .setLinearHeadingInterpolation(wallIntakePose.getHeading(), farShootPose.getHeading())
                .build();
        shootToFarIntake = follower.pathBuilder()
                .addPath(new BezierLine(farShootPose, farIntakePose))
                .setConstantHeadingInterpolation(farIntakePose.getHeading())
                .build();
        farIntakeToShoot = follower.pathBuilder()
                .addPath(new BezierLine(farIntakePose, farShootPose))
                .setLinearHeadingInterpolation(farIntakePose.getHeading(), farShootPose.getHeading())
                .build();

    }



}
