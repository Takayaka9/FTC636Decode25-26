package org.firstinspires.ftc.teamcode.scrims.ComandBase;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.scrims.RobotScrims;

public class ScrimColorTester extends OpMode {
    RobotScrims robot;
    TelemetryManager telemetryManager;
    @Override
    public void init() {
        robot = new RobotScrims(hardwareMap);
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void loop() {
        telemetryManager.update();

        robot.getDetectedColor(telemetryManager);
    }
}
