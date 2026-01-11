//package org.firstinspires.ftc.teamcode.RIstates.management.handlers.fsm.states.controllers.subsystems;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.TelemetryManager;
//import com.qualcomm.hardware.rev.RevColorSensorV3;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.NormalizedRGBA;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//
//@Configurable
//public class SimpleColor {
//    public RevColorSensorV3 sensor;
//    NormalizedRGBA colors = null;
//    private static double noBallDistance;
//
//    TelemetryManager telemetryM;
//    public SimpleColor(HardwareMap hardwareMap, TelemetryManager telemetryM) {
//        sensor = hardwareMap.get(RevColorSensorV3 .class, "colorSensor");
//        sensor.setGain(1);
//        this.telemetryM = telemetryM;
//    }
//
//    public enum DetectedColor{
//        GREEN,
//        PURPLE,
//        UNKNOWN
//    }
//
//
//    //not currently useful
//    public SimpleColor.DetectedColor updateDetectedColor(){
//        NormalizedRGBA colors = sensor.getNormalizedColors();
//
//        float normRed, normGreen, normBlue;
//
//        normRed = colors.red / colors.alpha;
//        normGreen = colors.green / colors.alpha;
//        normBlue = colors.blue / colors.alpha;
//
//        //TODO: calibrate the thresholds and add if statements
//
//        telemetryM.addData("Red", normRed);
//        telemetryM.addData("Green", normGreen);
//        telemetryM.addData("Blue", normBlue);
//
//        return SimpleColor.DetectedColor.UNKNOWN;
//    }
//
//    /**
//    detects if a ball is present using the distance measurement from the color sensor
//    returns true if a ball is present, false otherwise
//    no inputs
//     */
//    public boolean updateIsBallPresent() {
//        if (sensor.getDistance(DistanceUnit.CM) > noBallDistance){
//            return true;
//        } else return false;
//    }
//
//
//
//}
