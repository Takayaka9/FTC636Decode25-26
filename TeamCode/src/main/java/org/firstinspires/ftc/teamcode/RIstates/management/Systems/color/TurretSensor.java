package org.firstinspires.ftc.teamcode.RIstates.management.Systems.color;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class TurretSensor extends DistanceCheckBase implements IsBallPresent{
    public RevColorSensorV3 sensor;
    private static double noBallDistance = 25;

    public TurretSensor(HardwareMap hardwareMap) {
//        Init(hardwareMap, telemetryM, "tCS", sensor);
        sensor = hardwareMap.get(RevColorSensorV3.class, "tCS");
        sensor.setGain(1);
    }

    @Override
    public detectedLocation CheckIsBallPresent(){
        if (checkSensor(sensor, noBallDistance)){
            return detectedLocation.TURRET;
        } else {
            return detectedLocation.TurretCLEAR;
        }
    }

    @Override
    public double test() {
        return sensor.getDistance(DistanceUnit.MM);
    }
}
