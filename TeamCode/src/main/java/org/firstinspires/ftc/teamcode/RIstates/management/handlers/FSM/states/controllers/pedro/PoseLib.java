package org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.states.controllers.pedro;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class PoseLib {
    public static int farStartX = 89; public static int farStartY = 9; public static int farStartR = 90;
    public static int farShootX = 87; public static int farShootY = 15; public static int farShootR = 73;
    public static int nearStartX = 89; public static int nearStartY = 9; public static int nearStartR = 90;
    public static int nearShootX = 95; public static int nearShootY = 95; public static int nearShootR = 50;
    public static int PIntake3X = 100; public static int PIntake3Y = 35; public static int PIntake3R = 0;
    public static int Intake3X = 133; public static int Intake3Y = 35; public static int Intake3R = 0;
    public static int PIntake2X = 100; public static int PIntake2Y = 60; public static int PIntake2R = 0;
    public static int Intake2X = 133; public static int Intake2Y = 60; public static int Intake2R = 0;
    public static int PIntake1X = 100; public static int PIntake1Y = 84; public static int PIntake1R = 0;
    public static int Intake1X = 126; public static int Intake1Y = 84; public static int Intake1R = 0;
    public static int emptyX = 131; public static int emptyY = 60; public static int emptyR = 30;
    // TODO: add leave poses, close and far

    public final Pose farStartPose = new Pose(farStartX, farStartY, farStartR);
    public final Pose farShootPose = new Pose(farShootX, farShootY, farShootR);
    public final Pose nearStartPose = new Pose(nearStartX, nearStartY, nearStartR);
    public final Pose nearShootPose = new Pose(nearShootX, nearShootY, nearShootR);
    public final Pose pIntake3Pose = new Pose(PIntake3X, PIntake3Y, PIntake3R);
    public final Pose intake3Pose = new Pose(Intake3X, Intake3Y, Intake3R);
    public final Pose pIntake2Pose = new Pose(PIntake2X, PIntake2Y, PIntake2R);
    public final Pose intake2Pose = new Pose(Intake2X, Intake2Y, Intake2R);
    public final Pose pIntake1Pose = new Pose(PIntake1X, PIntake1Y, PIntake1R);
    public final Pose intake1Pose = new Pose(Intake1X, Intake1Y, Intake1R);
    public final Pose emptyPose = new Pose(emptyX, emptyY, emptyR);
    

}
