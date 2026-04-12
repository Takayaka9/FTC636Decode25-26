package org.firstinspires.ftc.teamcode.nePremier.utils.pedroUtils;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.nePremier.pedro.autoConstants.BluePoseLib;
import org.firstinspires.ftc.teamcode.nePremier.pedro.autoConstants.RedPoseLib;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.Alliance;

public class BasePathChain {
    public final Follower follower;
    public final Pose farStartPose;
    public final Pose farShootPose;
    public final Pose farIntakePose;
    public final Pose nearStartPose;
    public final Pose nearShootPose;
    public final Pose intakeP3Pose;
    public final Pose intake3Pose;
    public final Pose intakeP2Pose;
    public final Pose intake2Pose;
    public final Pose intakeP1Pose;
    public final Pose intake1Pose;
    public final Pose emptyPose;
    public final Pose emptyPPose;
    public final Pose wallIntakePPose;
    public final Pose wallIntakePose;
    public final Pose gatePose;
    public final Pose gateControlPose;
    public final Pose curvySpike2Control1Pose;
    public final Pose curvySpike2Control2Pose;
    public final Pose curvySpike3Control1Pose;
    public final Pose curvySpike3Control2Pose;
    public final Pose curvyGateControl1Pose;
    public final Pose curvyGateControl2Pose;
    public final Pose curvyGateOpenControlPose;
    public final Pose curvyGateOpenPose;
    public final Pose finalShootPose;
    public final Pose wallControlPose;
    public BasePathChain(Alliance alliance, Follower follower) {
        this.follower = follower;
        switch (alliance) {
            case RED:
                farStartPose = RedPoseLib.farStartPose;
                farShootPose = RedPoseLib.farShootPose;
                farIntakePose = RedPoseLib.farIntakePose;
                nearStartPose = RedPoseLib.nearStartPose;
                nearShootPose = RedPoseLib.nearShootPose;
                intakeP3Pose = RedPoseLib.intakeP3Pose;
                intake3Pose = RedPoseLib.intake3Pose;
                intakeP2Pose = RedPoseLib.intakeP2Pose;
                intake2Pose = RedPoseLib.intake2Pose;
                intakeP1Pose = RedPoseLib.intakeP1Pose;
                intake1Pose = RedPoseLib.intake1Pose;
                emptyPose = RedPoseLib.emptyPose;
                emptyPPose = RedPoseLib.emptyPPose;
                wallIntakePPose = RedPoseLib.wallIntakePPose;
                wallIntakePose = RedPoseLib.wallIntakePose;
                gatePose = RedPoseLib.gatePose;
                gateControlPose = RedPoseLib.gateControlPose;
                curvySpike2Control1Pose = RedPoseLib.curvySpike2Control1Pose;
                curvySpike2Control2Pose = RedPoseLib.curvySpike2Control2Pose;
                curvySpike3Control1Pose = RedPoseLib.curvySpike3Control1Pose;
                curvySpike3Control2Pose = RedPoseLib.curvySpike3Control2Pose;
                curvyGateControl1Pose = RedPoseLib.curvyGateControl1Pose;
                curvyGateControl2Pose = RedPoseLib.curvyGateControl2Pose;
                curvyGateOpenControlPose = RedPoseLib.curvyGateOpenControlPose;
                curvyGateOpenPose = RedPoseLib.curvyGateOpenPose;
                finalShootPose = RedPoseLib.finalShootPose;
                wallControlPose = RedPoseLib.wallControlPose;
                break;
            case BLUE:
                farStartPose = BluePoseLib.farStartPose;
                farShootPose = BluePoseLib.farShootPose;
                farIntakePose = BluePoseLib.farIntakePose;
                nearStartPose = BluePoseLib.nearStartPose;
                nearShootPose = BluePoseLib.nearShootPose;
                intakeP3Pose = BluePoseLib.intakeP3Pose;
                intake3Pose = BluePoseLib.intake3Pose;
                intakeP2Pose = BluePoseLib.intakeP2Pose;
                intake2Pose = BluePoseLib.intake2Pose;
                intakeP1Pose = BluePoseLib.intakeP1Pose;
                intake1Pose = BluePoseLib.intake1Pose;
                emptyPose = BluePoseLib.emptyPose;
                emptyPPose = BluePoseLib.emptyPPose;
                wallIntakePPose = BluePoseLib.wallIntakePPose;
                wallIntakePose = BluePoseLib.wallIntakePose;
                gatePose = BluePoseLib.gatePose;
                gateControlPose = BluePoseLib.gateControlPose;
                curvySpike2Control1Pose = BluePoseLib.curvySpike2Control1Pose;
                curvySpike2Control2Pose = BluePoseLib.curvySpike2Control2Pose;
                curvySpike3Control1Pose = BluePoseLib.curvySpike3Control1Pose;
                curvySpike3Control2Pose = BluePoseLib.curvySpike3Control2Pose;
                curvyGateControl1Pose = BluePoseLib.curvyGateControl1Pose;
                curvyGateControl2Pose = BluePoseLib.curvyGateControl2Pose;
                curvyGateOpenControlPose = BluePoseLib.curvyGateOpenControlPose;
                curvyGateOpenPose = BluePoseLib.curvyGateOpenPose;
                finalShootPose = BluePoseLib.finalShootPose;
                wallControlPose = BluePoseLib.wallControlPose;
                break;
            default:
                farStartPose = null;
                farShootPose = null;
                farIntakePose = null;
                nearStartPose = null;
                nearShootPose = null;
                intakeP3Pose = null;
                intake3Pose = null;
                intakeP2Pose = null;
                intake2Pose = null;
                intakeP1Pose = null;
                intake1Pose = null;
                emptyPose = null;
                emptyPPose = null;
                wallIntakePPose = null;
                wallIntakePose = null;
                gatePose = null;
                gateControlPose = null;
                curvySpike2Control1Pose = null;
                curvySpike2Control2Pose = null;
                curvySpike3Control1Pose = null;
                curvySpike3Control2Pose = null;
                curvyGateControl1Pose = null;
                curvyGateControl2Pose = null;
                curvyGateOpenControlPose = null;
                curvyGateOpenPose = null;
                finalShootPose = null;
                wallControlPose = null;
        }
    }


}
