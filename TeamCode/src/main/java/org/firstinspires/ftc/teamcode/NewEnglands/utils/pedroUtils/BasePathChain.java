package org.firstinspires.ftc.teamcode.NewEnglands.utils.pedroUtils;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.NewEnglands.pedro.poseLibs.BluePoseLib;
import org.firstinspires.ftc.teamcode.NewEnglands.pedro.poseLibs.RedPoseLib;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.alliance.Alliance;

public abstract class BasePathChain {
    private final BluePoseLib bluePoseLib;
    private final RedPoseLib redPoseLib;
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
    public void buildPaths() {}
    public BasePathChain(Alliance alliance, Follower follower) {
        this.follower = follower;
        redPoseLib = new RedPoseLib();
        bluePoseLib = new BluePoseLib();
        switch (alliance) {
            case RED:
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
                farStartPose = bluePoseLib.farStartPose;
                farShootPose = bluePoseLib.farShootPose;
                nearStartPose = bluePoseLib.nearStartPose;
                nearShootPose = bluePoseLib.nearShootPose;
                pIntake3Pose = bluePoseLib.pIntake3Pose;
                intake3Pose = bluePoseLib.intake3Pose;
                pIntake2Pose = bluePoseLib.pIntake2Pose;
                intake2Pose = bluePoseLib.intake2Pose;
                pIntake1Pose = bluePoseLib.pIntake1Pose;
                intake1Pose = bluePoseLib.intake1Pose;
                emptyPose = bluePoseLib.emptyPose;
                farLeavePose = bluePoseLib.farLeavePose;
                closeLeavePose = bluePoseLib.nearLeavePose;
        }
    }


}
