package org.firstinspires.ftc.teamcode.zscrims;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Disabled
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
