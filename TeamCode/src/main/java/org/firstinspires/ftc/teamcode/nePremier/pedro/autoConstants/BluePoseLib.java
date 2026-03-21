package org.firstinspires.ftc.teamcode.nePremier.pedro.autoConstants;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public final class BluePoseLib {
    public static final Pose farStartPose = RedPoseLib.farStartPose.getPose().mirror();
    public static final Pose farShootPose = new Pose(farShootX, farShootY, Math.toRadians(farShootR));
    public static final Pose nearStartPose = new Pose(nearStartX, nearStartY, Math.toRadians(nearStartR));
    public static final Pose nearShootPose = new Pose(nearShootX, nearShootY, Math.toRadians(nearShootR));
    public static final Pose pIntake3Pose = new Pose(PIntake3X, PIntake3Y, Math.toRadians(PIntake3R));
    public static final Pose intake3Pose = new Pose(Intake3X, Intake3Y, Math.toRadians(Intake3R));
    public static final Pose pIntake2Pose = new Pose(PIntake2X, PIntake2Y, Math.toRadians(PIntake2R));
    public static final Pose intake2Pose = new Pose(Intake2X, Intake2Y, Math.toRadians(Intake2R));
    public static final Pose pIntake1Pose = new Pose(PIntake1X, PIntake1Y, Math.toRadians(PIntake1R));
    public static final Pose intake1Pose = new Pose(Intake1X, Intake1Y, Math.toRadians(Intake1R));
    public static final Pose emptyPose = new Pose(emptyX, emptyY, Math.toRadians(emptyR));
}
