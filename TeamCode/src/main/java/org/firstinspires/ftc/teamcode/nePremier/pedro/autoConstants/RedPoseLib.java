package org.firstinspires.ftc.teamcode.nePremier.pedro.autoConstants;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@SuppressWarnings("NonFinalStaticVariableUsedInClassInitialization")
@Configurable
public final class RedPoseLib {
    public static int farStartX = 89; public static int farStartY = 8; public static int farStartR = 90;
    public static int farShootX = 87; public static int farShootY = 15; public static int farShootR = 73;
    public static int farIntakeX = 135; public static int farIntakeY = 15; public static int farIntakeR = 0;
    public static int nearStartX = 120; public static int nearStartY = 129; public static int nearStartR = 37;
    public static int nearShootX = 95; public static int nearShootY = 95; public static int nearShootR = 0;
    public static int intakeP3X = 80; public static int intakeP3Y = 27; public static int intakeP3R = 0;
    public static int Intake3X = 133; public static int Intake3Y = 35; public static int Intake3R = 0;
    public static int intakeP2X = 95; public static int intakeP2Y = 60; public static int intakeP2R = 0;
    public static int Intake2X = 134; public static int Intake2Y = 63; public static int Intake2R = 0;
    public static int intakeP1X = 95; public static int intakeP1Y = 84; public static int intakeP1R = 0;
    public static int Intake1X = 128; public static int Intake1Y = 86; public static int Intake1R = 355;
    public static int emptyX = 131; public static int emptyY = 64; public static int emptyR = 90;
    public static int emptyPX = 108; public static int emptyPY = 61; public static int emptyPR = 90;
    public static int gateX = 131; public static int gateY = 55; public static int gateR = 26;
    public static int gateControlX = 103; public static int gateControlY = 61; public static int gateControlR = 0;
    public static int curvySpike2Control1X = 95; public static int curvySpike2Control1Y = 79; public static int curvySpike2Control1R = 0;
    public static int curvySpike2Control2X = 65; public static int curvySpike2Control2Y = 59; public static int curvySpike2Control2R = 0;
    public static int curvySpike3Control1X = 110; public static int curvySpike3Control1Y = 78; public static int curvySpike3Control1R = 0; //unused
    public static int curvySpike3Control2X = 70; public static int curvySpike3Control2Y = 10; public static int curvySpike3Control2R = 0; //unused
    public static int curvyGateControl1X = 125; public static int curvyGateControl1Y = 82; public static int curvyGateControl1R = 0;
    public static int curvyGateControl2X = 105; public static int curvyGateControl2Y = 56; public static int curvyGateControl2R = 0;
    public static int finalShootX = 87; public static int finalShootY = 111; public static int finalShootR = 0;
    public static int wallIntakePX = 135; public static int wallIntakePY = 26; public static int wallIntakePR = 270;
    public static int wallIntakeX = 135; public static int wallIntakeY = 9; public static int wallIntakeR = 270;
    public static int wallControlX = 118; public static int wallControlY = 15; public static int wallControlR = 0;

    public static Pose farStartPose = new Pose(farStartX, farStartY, Math.toRadians(farStartR));
    public static Pose farShootPose = new Pose(farShootX, farShootY, Math.toRadians(farShootR));
    public static Pose farIntakePose = new Pose(farIntakeX, farIntakeY, Math.toRadians(farIntakeR));
    public static Pose nearStartPose = new Pose(nearStartX, nearStartY, Math.toRadians(nearStartR));
    public static Pose nearShootPose = new Pose(nearShootX, nearShootY, Math.toRadians(nearShootR));
    public static Pose intakeP3Pose = new Pose(intakeP3X, intakeP3Y, Math.toRadians(intakeP3R));
    public static Pose intake3Pose = new Pose(Intake3X, Intake3Y, Math.toRadians(Intake3R));
    public static Pose intakeP2Pose = new Pose(intakeP2X, intakeP2Y, Math.toRadians(intakeP2R));
    public static Pose intake2Pose = new Pose(Intake2X, Intake2Y, Math.toRadians(Intake2R));
    public static Pose intakeP1Pose = new Pose(intakeP1X, intakeP1Y, Math.toRadians(intakeP1R));
    public static Pose intake1Pose = new Pose(Intake1X, Intake1Y, Math.toRadians(Intake1R));
    public static Pose emptyPose = new Pose(emptyX, emptyY, Math.toRadians(emptyR));
    public static Pose emptyPPose = new Pose(emptyPX, emptyPY, Math.toRadians(emptyPR));
    public static Pose gatePose = new Pose(gateX, gateY, Math.toRadians(gateR));
    public static Pose gateControlPose = new Pose(gateControlX, gateControlY, Math.toRadians(gateControlR));
    public static Pose curvySpike2Control1Pose = new Pose(curvySpike2Control1X, curvySpike2Control1Y, Math.toRadians(curvySpike2Control1R));
    public static Pose curvySpike2Control2Pose = new Pose(curvySpike2Control2X, curvySpike2Control2Y, Math.toRadians(curvySpike2Control2R));
    public static Pose curvySpike3Control1Pose = new Pose(curvySpike3Control1X, curvySpike3Control1Y, Math.toRadians(curvySpike3Control1R));
    public static Pose curvySpike3Control2Pose = new Pose(curvySpike3Control2X, curvySpike3Control2Y, Math.toRadians(curvySpike3Control2R));
    public static Pose curvyGateControl1Pose = new Pose(curvyGateControl1X, curvyGateControl1Y, Math.toRadians(curvyGateControl1R));
    public static Pose curvyGateControl2Pose = new Pose(curvyGateControl2X, curvyGateControl2Y, Math.toRadians(curvyGateControl2R));
    public static Pose finalShootPose = new Pose(finalShootX, finalShootY, Math.toRadians(finalShootR));
    public static Pose wallIntakePPose = new Pose(wallIntakePX, wallIntakePY, Math.toRadians(wallIntakePR));
    public static Pose wallIntakePose = new Pose(wallIntakeX, wallIntakeY, Math.toRadians(wallIntakeR));
    public static Pose wallControlPose = new Pose(wallControlX, wallControlY, Math.toRadians(wallControlR));
}
