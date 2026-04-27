package org.firstinspires.ftc.teamcode.afterPremier.util.pathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.afterPremier.util.Alliance;

@Configurable
public class PoseLib {
    public PoseLib(Alliance a) {
        if (a == Alliance.BLUE) {
            farStartPose = farStartPose.getPose().mirror();
            farShootPose = farShootPose.getPose().mirror();
            farIntakePose = farIntakePose.getPose().mirror();
            nearStartPose = nearStartPose.getPose().mirror();
            nearShootPose = nearShootPose.getPose().mirror();
            intakeP3Pose = intakeP3Pose.getPose().mirror();
            intake3Pose = intake3Pose.getPose().mirror();
            intakeP2Pose = intakeP2Pose.getPose().mirror();
            intake2Pose = intake2Pose.getPose().mirror();
            intakeP1Pose = intakeP1Pose.getPose().mirror();
            intake1Pose = intake1Pose.getPose().mirror();
            emptyPose = emptyPose.getPose().mirror();
            emptyPPose = emptyPPose.getPose().mirror();
            gatePose = gatePose.getPose().mirror();
            gateControlPose = gateControlPose.getPose().mirror();
            curvySpike2Control1Pose = curvySpike2Control1Pose.getPose().mirror();
            curvySpike2Control2Pose = curvySpike2Control2Pose.getPose().mirror();
            curvySpike3Control1Pose = curvySpike3Control1Pose.getPose().mirror();
            curvySpike3Control2Pose = curvySpike3Control2Pose.getPose().mirror();
            curvyGateControl1Pose = curvyGateControl1Pose.getPose().mirror();
            curvyGateControl2Pose = curvyGateControl2Pose.getPose().mirror();
            finalShootPose = finalShootPose.getPose().mirror();
            wallIntakePPose = wallIntakePPose.getPose().mirror();
            wallIntakePose = wallIntakePose.getPose().mirror();
            wallControlPose = wallControlPose.getPose().mirror();
        }
    }

    public Pose farStartPose = new Pose(89, 8, Math.toRadians(90));
    public Pose farShootPose = new Pose(87, 15, Math.toRadians(73));
    public Pose farIntakePose = new Pose(135, 15, Math.toRadians(0));
    public Pose nearStartPose = new Pose(120, 129, Math.toRadians(37));
    public Pose nearShootPose = new Pose(95, 95, Math.toRadians(0));
    public Pose intakeP3Pose = new Pose(80, 27, Math.toRadians(0));
    public Pose intake3Pose = new Pose(133, 35, Math.toRadians(0));
    public Pose intakeP2Pose = new Pose(95, 60, Math.toRadians(0));
    public Pose intake2Pose = new Pose(134, 63, Math.toRadians(0));
    public Pose intakeP1Pose = new Pose(95, 84, Math.toRadians(0));
    public Pose intake1Pose = new Pose(128, 86, Math.toRadians(355));
    public Pose emptyPose = new Pose(131, 67, Math.toRadians(90));
    public Pose emptyPPose = new Pose(114, 65, Math.toRadians(90));
    public Pose gatePose = new Pose(131, 61, Math.toRadians(26));
    public Pose gateControlPose = new Pose(96, 61, Math.toRadians(0));
    public Pose curvySpike2Control1Pose = new Pose(95, 79, Math.toRadians(0));
    public Pose curvySpike2Control2Pose = new Pose(53, 59, Math.toRadians(0));
    public Pose curvySpike3Control1Pose = new Pose(110, 78, Math.toRadians(0));
    public Pose curvySpike3Control2Pose = new Pose(70, 10, Math.toRadians(0));
    public Pose curvyGateControl1Pose = new Pose(125, 82, Math.toRadians(0));
    public Pose curvyGateControl2Pose = new Pose(105, 56, Math.toRadians(0));
    public Pose finalShootPose = new Pose(87, 111, Math.toRadians(0));
    public Pose wallIntakePPose = new Pose(135, 26, Math.toRadians(270));
    public Pose wallIntakePose = new Pose(135, 9, Math.toRadians(270));
    public Pose wallControlPose = new Pose(118, 15, Math.toRadians(0));
}
