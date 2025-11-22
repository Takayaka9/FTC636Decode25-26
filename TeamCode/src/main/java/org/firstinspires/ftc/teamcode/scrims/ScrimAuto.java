package org.firstinspires.ftc.teamcode.scrims;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

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

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        waitForStart();

        while(opModeIsActive()){
            telemetryM.update();
            follower.update();

            CommandScheduler commandScheduler;


        }
    }

    public static double startX = 48;
    public static double startY = 135;
    public static double scoreX = 65;
    public static double scoreY = 90;
    public static double scoreA = 136;

    private final Pose startPose = new Pose(startX, startY, Math.toRadians(90));
    private final Pose scorePose = new Pose(scoreX, scoreY, Math.toRadians(scoreA));

    private Path scorePreload;
    public void buildPaths(){
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
    }
}
