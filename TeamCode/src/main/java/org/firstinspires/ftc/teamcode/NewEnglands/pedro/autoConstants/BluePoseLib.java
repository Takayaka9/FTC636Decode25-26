package org.firstinspires.ftc.teamcode.NewEnglands.pedro.autoConstants;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public abstract class BluePoseLib {
    private static final int farStartX = 55; private static final int farStartY = 9; private static final int farStartR = 90;
    private static final int farShootX = 57; private static final int farShootY = 15; private static final int farShootR = 107;
    private static final int nearStartX = 24; private static final int nearStartY = 129; private static final int nearStartR = 143;
    private static final int nearShootX = 49; private static final int nearShootY = 95; private static final int nearShootR = 180;
    private static final int PIntake3X = 40; private static final int PIntake3Y = 35; private static final int PIntake3R = 180;
    private static final int Intake3X = 11; private static final int Intake3Y = 35; private static final int Intake3R = 180;
    private static final int PIntake2X = 50; private static final int PIntake2Y = 60; private static final int PIntake2R = 180;
    private static final int Intake2X = 11; private static final int Intake2Y = 60; private static final int Intake2R = 180;
    private static final int PIntake1X = 40; private static final int PIntake1Y = 84; private static final int PIntake1R = 180;
    private static final int Intake1X = 18; private static final int Intake1Y = 84; private static final int Intake1R = 180;
    private static final int emptyX = 13; private static final int emptyY = 60; private static final int emptyR = 150;
    private static final int farLeaveX = 40; private static final int farLeaveY = 55; private static final int farLeaveR = 135;
    private static final int closeLeaveX = 40; private static final int closeLeaveY = 55; private static final int closeLeaveR = 135;

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
