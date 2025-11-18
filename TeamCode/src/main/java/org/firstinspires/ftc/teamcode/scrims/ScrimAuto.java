package org.firstinspires.ftc.teamcode.scrims;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.scrims.ComandBase.IntakeSubsystem;

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
}
