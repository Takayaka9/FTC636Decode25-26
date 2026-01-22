package org.firstinspires.ftc.teamcode.RIstates.management.pedro;

import com.pedropathing.geometry.Pose;

public class BluePoseLib {
    public static int farStartX = 55; public static int farStartY = 9; public static int farStartR = 90;
    public static int farShootX = 57; public static int farShootY = 15; public static int farShootR = 107;
    public static int nearStartX = 55; public static int nearStartY = 9; public static int nearStartR = 90;
    public static int nearShootX = 49; public static int nearShootY = 95; public static int nearShootR = 130;
    public static int PIntake3X = 44; public static int PIntake3Y = 35; public static int PIntake3R = 180;
    public static int Intake3X = 11; public static int Intake3Y = 35; public static int Intake3R = 180;
    public static int PIntake2X = 44; public static int PIntake2Y = 60; public static int PIntake2R = 180;
    public static int Intake2X = 11; public static int Intake2Y = 60; public static int Intake2R = 180;
    public static int PIntake1X = 44; public static int PIntake1Y = 84; public static int PIntake1R = 180;
    public static int Intake1X = 18; public static int Intake1Y = 84; public static int Intake1R = 180;
    public static int emptyX = 13; public static int emptyY = 60; public static int emptyR = 150;
    public static int farLeaveX = 51; public static int farLeaveY = 23; public static int farLeaveR = 135;
    public static int closeLeaveX = 33; public static int closeLeaveY = 96; public static int closeLeaveR = 135;

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
    public final Pose farLeavePose = new Pose(farLeaveX, farLeaveY, farLeaveR);
    public final Pose closeLeavePose = new Pose(closeLeaveX, closeLeaveY, closeLeaveR);
}
