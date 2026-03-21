package org.firstinspires.ftc.teamcode.nePremier.pedro.autoConstants;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@SuppressWarnings("NonFinalStaticVariableUsedInClassInitialization")
@Configurable
public final class RedPoseLib {
    public static int farStartX = 89; public static int farStartY = 8; public static int farStartR = 90;
    public static int farShootX = 87; public static int farShootY = 15; public static int farShootR = 73;
    public static int nearStartX = 120; public static int nearStartY = 129; public static int nearStartR = 37;
    public static int nearShootX = 95; public static int nearShootY = 95; public static int nearShootR = 0;
    public static int PIntake3X = 95; public static int PIntake3Y = 35; public static int PIntake3R = 0;
    public static int Intake3X = 133; public static int Intake3Y = 35; public static int Intake3R = 0;
    public static int PIntake2X = 95; public static int PIntake2Y = 62; public static int PIntake2R = 0;
    public static int Intake2X = 125; public static int Intake2Y = 62; public static int Intake2R = 0;
    public static int PIntake1X = 95; public static int PIntake1Y = 87; public static int PIntake1R = 0;
    public static int Intake1X = 126; public static int Intake1Y = 87; public static int Intake1R = 0;
    public static int emptyX = 131; public static int emptyY = 60; public static int emptyR = 90;


    public static final Pose farStartPose = new Pose(farStartX, farStartY, Math.toRadians(farStartR));
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
