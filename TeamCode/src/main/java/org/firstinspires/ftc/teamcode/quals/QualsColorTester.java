package org.firstinspires.ftc.teamcode.quals;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.scrims.RobotScrims;

@Configurable
@TeleOp(name = "ColorSensor Test", group = "TeleOp")
public class QualsColorTester extends OpMode {
    RobotQuals robot;
    TelemetryManager telemetryManager;
    @Override
    public void init() {
        robot = new RobotQuals(hardwareMap);
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void loop() {
        telemetryManager.update();

        robot.getDetectedColor(telemetryManager);
    }
}
