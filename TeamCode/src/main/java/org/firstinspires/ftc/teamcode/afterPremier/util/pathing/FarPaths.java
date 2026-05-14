package org.firstinspires.ftc.teamcode.afterPremier.util.pathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.afterPremier.util.Alliance;

public class FarPaths {
    Follower f;
    PoseLib p;
    public FarPaths(Alliance a, Follower f) {
        p = new PoseLib(a);
        this.f = f;
        buildPaths();
    }
    public PathChain startToShoot,
            intakeSpike3,
            spike3toShoot,
            shootToWall,
            wallToShoot,
            shootToFarIntake,
            farIntakeToShoot;
    public void buildPaths() {
        f.pathBuilder().setGlobalTangentHeadingInterpolation()
                .setGlobalDeceleration();

        startToShoot = f.pathBuilder()
                .addPath(new BezierLine(p.farStartPose, p.farShootPose))
                .setLinearHeadingInterpolation(p.farStartPose.getHeading(), p.farShootPose.getHeading(), 0.4)
                .build();

        intakeSpike3 = f.pathBuilder()
                .addPath(new BezierLine(p.farStartPose, p.intakeP3Pose))
                .setLinearHeadingInterpolation(p.farStartPose.getHeading(), p.intakeP3Pose.getHeading(), 0.6)
                .addPath(new BezierLine(p.intakeP3Pose, p.intake3Pose))
                .build();

        spike3toShoot = f.pathBuilder()
                .addPath(new BezierLine(p.intake3Pose, p.farShootPose))
                .setConstantHeadingInterpolation(p.farShootPose.getHeading())
                .build();

        shootToWall = f.pathBuilder()
                .addPath(new BezierLine(p.farShootPose, p.wallIntakePPose))
                .setLinearHeadingInterpolation(p.farShootPose.getHeading(), p.wallIntakePPose.getHeading())
                .addPath(new BezierLine(p.wallIntakePPose, p.wallIntakePose))
                .setConstantHeadingInterpolation(p.wallIntakePPose.getHeading())
                .build();

        wallToShoot = f.pathBuilder()
                .addPath(new BezierCurve(p.wallIntakePose, p.wallControlPose, p.farShootPose))
                .setLinearHeadingInterpolation(p.wallIntakePose.getHeading(), p.farShootPose.getHeading())
                .build();

        shootToFarIntake = f.pathBuilder()
                .addPath(new BezierLine(p.farShootPose, p.farIntakePose))
                .setConstantHeadingInterpolation(p.farShootPose.getHeading())
                .build();

        farIntakeToShoot = f.pathBuilder()
                .addPath(new BezierLine(p.farIntakePose, p.farShootPose))
                .setConstantHeadingInterpolation(p.farShootPose.getHeading())
                .build();
    }
    public Pose getStartingPose(){
        return p.farStartPose;
    }
}
