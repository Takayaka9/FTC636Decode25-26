package org.firstinspires.ftc.teamcode.nePremier.utils.pedroUtils;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.nePremier.pedro.autoConstants.BluePoseLib;
import org.firstinspires.ftc.teamcode.nePremier.pedro.autoConstants.RedPoseLib;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.Alliance;

public abstract class BasePathChain {
    public final Follower follower;
    public Pose farStartPose;
    public Pose farShootPose;
    public Pose nearStartPose;
    public Pose nearShootPose;
    public Pose pIntake3Pose;
    public Pose intake3Pose;
    public Pose pIntake2Pose;
    public Pose intake2Pose;
    public Pose pIntake1Pose;
    public Pose intake1Pose;
    public Pose emptyPose;
    public Pose farLeavePose;
    public Pose closeLeavePose;
    public BasePathChain(Alliance alliance, Follower follower) {
        this.follower = follower;
        switch (alliance) {
            case RED:
                farStartPose = RedPoseLib.farStartPose;
                farShootPose = RedPoseLib.farShootPose;
                nearStartPose = RedPoseLib.nearStartPose;
                nearShootPose = RedPoseLib.nearShootPose;
                pIntake3Pose = RedPoseLib.pIntake3Pose;
                intake3Pose = RedPoseLib.intake3Pose;
                pIntake2Pose = RedPoseLib.pIntake2Pose;
                intake2Pose = RedPoseLib.intake2Pose;
                pIntake1Pose = RedPoseLib.pIntake1Pose;
                intake1Pose = RedPoseLib.intake1Pose;
                emptyPose = RedPoseLib.emptyPose;
                farLeavePose = RedPoseLib.farLeavePose;
                closeLeavePose = RedPoseLib.closeLeavePose;
                break;
            case BLUE:
                farStartPose = BluePoseLib.farStartPose;
                farShootPose = BluePoseLib.farShootPose;
                nearStartPose = BluePoseLib.nearStartPose;
                nearShootPose = BluePoseLib.nearShootPose;
                pIntake3Pose = BluePoseLib.pIntake3Pose;
                intake3Pose = BluePoseLib.intake3Pose;
                pIntake2Pose = BluePoseLib.pIntake2Pose;
                intake2Pose = BluePoseLib.intake2Pose;
                pIntake1Pose = BluePoseLib.pIntake1Pose;
                intake1Pose = BluePoseLib.intake1Pose;
                emptyPose = BluePoseLib.emptyPose;
                farLeavePose = BluePoseLib.farLeavePose;
                closeLeavePose = BluePoseLib.nearLeavePose;
                break;
            default:
                farStartPose = null;
                farShootPose = null;
                nearStartPose = null;
                nearShootPose = null;
                pIntake3Pose = null;
                intake3Pose = null;
                pIntake2Pose = null;
                intake2Pose = null;
                pIntake1Pose = null;
                intake1Pose = null;
                emptyPose = null;
                farLeavePose = null;
                closeLeavePose = null;
        }
    }


}
