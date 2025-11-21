package org.firstinspires.ftc.teamcode.scrims;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//pray
@Configurable
@Autonomous(name = "ScrimAuto", group = "Autonomous")
public class ScrimAuto extends LinearOpMode {
    Follower follower;
    TelemetryManager telemetryM;
    RobotScrims robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotScrims(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();


        waitForStart();

        while(opModeIsActive()){
            telemetryM.update();
            follower.update();
        }
    }

    public static double startX = 60;
    public static double startY = 0;
    public static double scoreX = 0;
    public static double scoreY = 0;
    public static double scoreA = 120;

    private final Pose startPose = new Pose(startX, startY, Math.toRadians(0));
    private final Pose scorePose = new Pose(scoreX, scoreY, Math.toRadians(scoreA));
}
