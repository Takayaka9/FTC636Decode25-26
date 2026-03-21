package org.firstinspires.ftc.teamcode.nePremier.pedro.autoConstants;

import com.pedropathing.geometry.Pose;

public final class BluePoseLib {
    public static final Pose farStartPose = RedPoseLib.farStartPose.getPose().mirror();
    public static final Pose farShootPose = RedPoseLib.farShootPose.getPose().mirror();
    public static final Pose nearStartPose = RedPoseLib.nearStartPose.getPose().mirror();
    public static final Pose nearShootPose = RedPoseLib.nearShootPose.getPose().mirror();
    public static final Pose pIntake3Pose = RedPoseLib.pIntake3Pose.getPose().mirror();
    public static final Pose intake3Pose = RedPoseLib.intake3Pose.getPose().mirror();
    public static final Pose pIntake2Pose = RedPoseLib.intake3Pose.getPose().mirror();
    public static final Pose intake2Pose = RedPoseLib.intake2Pose.getPose().mirror();
    public static final Pose pIntake1Pose = RedPoseLib.pIntake1Pose.getPose().mirror();
    public static final Pose intake1Pose = RedPoseLib.pIntake1Pose.getPose().mirror();
    public static final Pose emptyPose = RedPoseLib.pIntake1Pose.getPose().mirror();

}
