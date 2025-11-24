package org.firstinspires.ftc.teamcode.quals;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

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
        telemetryManager.addData("color", getDetectedColor());
        telemetryManager.update();
        //add something to return a string with "g" or "p" when color is green or purple
    }

    public enum DetectedColor{
        GREEN,
        PURPLE,
        UNKNOWN
    }
    public DetectedColor getDetectedColor(){
        NormalizedRGBA colors = robot.colorSensor.getNormalizedColors();

        float normRed, normGreen, normBlue;

        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;

        //TODO: calibrate the thresholds and add if statements

        telemetryManager.addData("Red", normRed);
        telemetryManager.addData("Green", normGreen);
        telemetryManager.addData("Blue", normBlue);

        return DetectedColor.UNKNOWN;
    }
}
