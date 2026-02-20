package org.firstinspires.ftc.teamcode.NewEnglands.utils.pedroUtils.interfaceUtils;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.NewEnglands.pedro.poseLibs.BluePoseLib;
import org.firstinspires.ftc.teamcode.NewEnglands.pedro.poseLibs.RedPoseLib;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.alliance.Alliance;

public abstract class PoseLibWrapper {
    BluePoseLib bluePoseLib;
    RedPoseLib redPoseLib;

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
    public PoseLibWrapper(Alliance alliance) {

        switch (alliance) {
            case RED:
                redPoseLib = new RedPoseLib();

                farStartPose = redPoseLib.farStartPose;
                farShootPose = redPoseLib.farShootPose;
                nearStartPose = redPoseLib.nearStartPose;
                nearShootPose = redPoseLib.nearShootPose;
                pIntake3Pose = redPoseLib.pIntake3Pose;
                intake3Pose = redPoseLib.intake3Pose;
                pIntake2Pose = redPoseLib.pIntake2Pose;
                intake2Pose = redPoseLib.intake2Pose;
                pIntake1Pose = redPoseLib.pIntake1Pose;
                intake1Pose = redPoseLib.intake1Pose;
                emptyPose = redPoseLib.emptyPose;
                farLeavePose = redPoseLib.farLeavePose;
                closeLeavePose = redPoseLib.closeLeavePose;

                break;
            case BLUE:
                bluePoseLib = new BluePoseLib();

                farStartPose = redPoseLib.farStartPose;
                farShootPose = redPoseLib.farShootPose;
                nearStartPose = redPoseLib.nearStartPose;
                nearShootPose = redPoseLib.nearShootPose;
                pIntake3Pose = redPoseLib.pIntake3Pose;
                intake3Pose = redPoseLib.intake3Pose;
                pIntake2Pose = redPoseLib.pIntake2Pose;
                intake2Pose = redPoseLib.intake2Pose;
                pIntake1Pose = redPoseLib.pIntake1Pose;
                intake1Pose = redPoseLib.intake1Pose;
                emptyPose = redPoseLib.emptyPose;
                farLeavePose = redPoseLib.farLeavePose;
                closeLeavePose = redPoseLib.closeLeavePose;
        }
    }
}
