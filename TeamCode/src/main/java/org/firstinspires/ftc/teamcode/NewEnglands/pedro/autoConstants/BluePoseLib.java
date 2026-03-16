package org.firstinspires.ftc.teamcode.NewEnglands.pedro.autoConstants;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public abstract class BluePoseLib {
    private static int farStartX = 55; private static int farStartY = 9; private static int farStartR = 90;
    private static int farShootX = 57; private static int farShootY = 15; private static int farShootR = 107;
    private static int nearStartX = 24; private static int nearStartY = 129; private static int nearStartR = 143;
    private static int nearShootX = 49; private static int nearShootY = 95; private static int nearShootR = 180;
    private static int PIntake3X = 40; private static int PIntake3Y = 35; private static int PIntake3R = 180;
    private static int Intake3X = 11; private static int Intake3Y = 35; private static int Intake3R = 180;
    private static int PIntake2X = 50; private static int PIntake2Y = 60; private static int PIntake2R = 180;
    private static int Intake2X = 11; private static int Intake2Y = 60; private static int Intake2R = 180;
    private static int PIntake1X = 40; private static int PIntake1Y = 84; private static int PIntake1R = 180;
    private static int Intake1X = 18; private static int Intake1Y = 84; private static int Intake1R = 180;
    private static int emptyX = 13; private static int emptyY = 60; private static int emptyR = 150;
    private static int farLeaveX = 40; private static int farLeaveY = 55; private static int farLeaveR = 135;
    private static int closeLeaveX = 40; private static int closeLeaveY = 55; private static int closeLeaveR = 135;

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
    public static final Pose farLeavePose = new Pose(farLeaveX, farLeaveY, Math.toRadians(farLeaveR));
    public static final Pose nearLeavePose = new Pose(closeLeaveX, closeLeaveY, Math.toRadians(closeLeaveR));
}
