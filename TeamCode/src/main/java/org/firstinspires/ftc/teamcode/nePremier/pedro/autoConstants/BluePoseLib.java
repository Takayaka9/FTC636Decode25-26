package org.firstinspires.ftc.teamcode.nePremier.pedro.autoConstants;

import com.pedropathing.geometry.Pose;

public final class BluePoseLib {
    public static final Pose farStartPose = RedPoseLib.farStartPose.getPose().mirror();
    public static final Pose farShootPose = RedPoseLib.farShootPose.getPose().mirror();
    public static final Pose nearStartPose = RedPoseLib.nearStartPose.getPose().mirror();
    public static final Pose nearShootPose = RedPoseLib.nearShootPose.getPose().mirror();
    public static final Pose intakeP3Pose = RedPoseLib.intakeP3Pose.getPose().mirror();
    public static final Pose intake3Pose = RedPoseLib.intake3Pose.getPose().mirror();
    public static final Pose intakeP2Pose = RedPoseLib.intake3Pose.getPose().mirror();
    public static final Pose intake2Pose = RedPoseLib.intake2Pose.getPose().mirror();
    public static final Pose intakeP1Pose = RedPoseLib.intakeP1Pose.getPose().mirror();
    public static final Pose intake1Pose = RedPoseLib.intakeP1Pose.getPose().mirror();
    public static final Pose emptyPose = RedPoseLib.intakeP1Pose.getPose().mirror();
    public static final Pose emptyPPose = RedPoseLib.emptyPPose.getPose().mirror();
}
