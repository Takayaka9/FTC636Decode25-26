package org.firstinspires.ftc.teamcode.afterPremier.util.pathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.afterPremier.util.Alliance;

@Configurable
public class PoseLib {
    public PoseLib(Alliance a) {
        if (a == Alliance.BLUE) {
            farStartPose = farStartPose.mirror();
            farShootPose = farShootPose.mirror();
            farIntakePose = farIntakePose.mirror();
            nearStartPose = nearStartPose.mirror();
            nearShootPose = nearShootPose.mirror();
            intakeP3Pose = intakeP3Pose.mirror();
            intake3Pose = intake3Pose.mirror();
            intakeP2Pose = intakeP2Pose.mirror();
            intake2Pose = intake2Pose.mirror();
            intakeP1Pose = intakeP1Pose.mirror();
            intake1Pose = intake1Pose.mirror();
            emptyPose = emptyPose.mirror();
            emptyPPose = emptyPPose.mirror();
            gatePose = gatePose.mirror();
            gateControlPose = gateControlPose.mirror();
            curvySpike2Control1Pose = curvySpike2Control1Pose.mirror();
            curvySpike2Control2Pose = curvySpike2Control2Pose.mirror();
            curvySpike3Control1Pose = curvySpike3Control1Pose.mirror();
            curvySpike3Control2Pose = curvySpike3Control2Pose.mirror();
            curvyGateControl1Pose = curvyGateControl1Pose.mirror();
            curvyGateControl2Pose = curvyGateControl2Pose.mirror();
            finalShootPose = finalShootPose.mirror();
            wallIntakePPose = wallIntakePPose.mirror();
            wallIntakePose = wallIntakePose.mirror();
            wallControlPose = wallControlPose.mirror();
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
