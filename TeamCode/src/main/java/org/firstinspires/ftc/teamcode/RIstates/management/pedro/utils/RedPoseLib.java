package org.firstinspires.ftc.teamcode.RIstates.management.pedro.utils;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public abstract class RedPoseLib {
    public static int farStartX = 89; public static int farStartY = 9; public static int farStartR = 90;
    public static int farShootX = 87; public static int farShootY = 15; public static int farShootR = 73;
    public static int nearStartX = 120; public static int nearStartY = 129; public static int nearStartR = 37;
    public static int nearShootX = 95; public static int nearShootY = 95; public static int nearShootR = 50;
    public static int PIntake3X = 100; public static int PIntake3Y = 35; public static int PIntake3R = 0;
    public static int Intake3X = 133; public static int Intake3Y = 35; public static int Intake3R = 0;
    public static int PIntake2X = 100; public static int PIntake2Y = 60; public static int PIntake2R = 0;
    public static int Intake2X = 133; public static int Intake2Y = 60; public static int Intake2R = 0;
    public static int PIntake1X = 100; public static int PIntake1Y = 84; public static int PIntake1R = 0;
    public static int Intake1X = 126; public static int Intake1Y = 84; public static int Intake1R = 0;
    public static int emptyX = 131; public static int emptyY = 60; public static int emptyR = 90;
    public static int farLeaveX = 93; public static int farLeaveY = 23; public static int farLeaveR = 45;
    public static int closeLeaveX = 111; public static int closeLeaveY = 96; public static int closeLeaveR = 45;


    public final Pose farStartPose = new Pose(farStartX, farStartY, Math.toRadians(farStartR));
    public final Pose farShootPose = new Pose(farShootX, farShootY, Math.toRadians(farShootR));
    public final Pose nearStartPose = new Pose(nearStartX, nearStartY, Math.toRadians(nearStartR));
    public final Pose nearShootPose = new Pose(nearShootX, nearShootY, Math.toRadians(nearShootR));
    public final Pose pIntake3Pose = new Pose(PIntake3X, PIntake3Y, Math.toRadians(PIntake3R));
    public final Pose intake3Pose = new Pose(Intake3X, Intake3Y, Math.toRadians(Intake3R));
    public final Pose pIntake2Pose = new Pose(PIntake2X, PIntake2Y, Math.toRadians(PIntake2R));
    public final Pose intake2Pose = new Pose(Intake2X, Intake2Y, Math.toRadians(Intake2R));
    public final Pose pIntake1Pose = new Pose(PIntake1X, PIntake1Y, Math.toRadians(PIntake1R));
    public final Pose intake1Pose = new Pose(Intake1X, Intake1Y, Math.toRadians(Intake1R));
    public final Pose emptyPose = new Pose(emptyX, emptyY, Math.toRadians(emptyR));
    public final Pose farLeavePose = new Pose(farLeaveX, farLeaveY, Math.toRadians(farLeaveR));
    public final Pose closeLeavePose = new Pose(closeLeaveX, closeLeaveY, Math.toRadians(closeLeaveR));


}
