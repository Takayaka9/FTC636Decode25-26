package org.firstinspires.ftc.teamcode.scrims;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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
