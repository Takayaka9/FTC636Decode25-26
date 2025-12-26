package org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.states.controllers.subsystems;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class Color {
    Config robot;
    public enum DetectedColor{
        GREEN,
        PURPLE,
        UNKNOWN
    }

    public DetectedColor getDetectedColor(TelemetryManager telemetryManager, HardwareMap hardwareMap){
        robot = new Config(hardwareMap);

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
