package org.firstinspires.ftc.teamcode.nePremier.pedro.autoConstants;

import com.pedropathing.geometry.Pose;

public final class BluePoseLib {
    public static Pose farStartPose = RedPoseLib.farStartPose.getPose().mirror();
    public static Pose farShootPose = RedPoseLib.farShootPose.getPose().mirror();
    public static Pose farIntakePose = RedPoseLib.farIntakePose.getPose().mirror();
    public static Pose nearStartPose = RedPoseLib.nearStartPose.getPose().mirror();
    public static Pose nearShootPose = RedPoseLib.nearShootPose.getPose().mirror();
    public static Pose intakeP3Pose = RedPoseLib.intakeP3Pose.getPose().mirror();
    public static Pose intake3Pose = RedPoseLib.intake3Pose.getPose().mirror();
    public static Pose intakeP2Pose = RedPoseLib.intakeP2Pose.getPose().mirror();
    public static Pose intake2Pose = RedPoseLib.intake2Pose.getPose().mirror();
    public static Pose intakeP1Pose = RedPoseLib.intakeP1Pose.getPose().mirror();
    public static Pose intake1Pose = RedPoseLib.intake1Pose.getPose().mirror();
    public static Pose emptyPose = RedPoseLib.emptyPose.getPose().mirror();
    public static Pose emptyPPose = RedPoseLib.emptyPPose.getPose().mirror();
    public static Pose gatePose = RedPoseLib.gatePose.getPose().mirror();
    public static Pose gateControlPose = RedPoseLib.gateControlPose.getPose().mirror();
    public static Pose finalShootPose = RedPoseLib.finalShootPose.getPose().mirror();
    public static Pose wallIntakePPose = RedPoseLib.wallIntakePPose.getPose().mirror();
    public static Pose wallIntakePose = RedPoseLib.wallIntakePose.getPose().mirror();
    public static Pose wallControlPose = RedPoseLib.wallControlPose.getPose().mirror();
}
