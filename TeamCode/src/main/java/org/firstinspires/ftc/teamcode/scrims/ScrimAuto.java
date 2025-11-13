package org.firstinspires.ftc.teamcode.scrims;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//pray
@Configurable
@Autonomous(name = "ScrimAuto", group = "Autonomous")
public class ScrimAuto extends LinearOpMode {
    Follower follower;
    TelemetryManager TelemetryM;
    RobotScrims robot = new RobotScrims(hardwareMap);
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
    }
}
