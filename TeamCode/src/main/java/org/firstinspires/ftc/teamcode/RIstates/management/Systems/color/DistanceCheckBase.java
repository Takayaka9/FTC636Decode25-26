package org.firstinspires.ftc.teamcode.RIstates.management.Systems.color;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

abstract class DistanceCheckBase implements IsBallPresent{
    protected final void Init(HardwareMap hardwareMap, TelemetryManager telemetryM, String name, RevColorSensorV3 sensor) {
//        sensor = hardwareMap.get(RevColorSensorV3.class, name);
//        sensor.setGain(1);
    }

    protected final boolean checkSensor(RevColorSensorV3 sensor, double noBallDistance) {
        if (sensor.getDistance(DistanceUnit.MM) > noBallDistance){
            return true;
        } else return false;
    }

}