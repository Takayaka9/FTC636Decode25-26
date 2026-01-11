package org.firstinspires.ftc.teamcode.RIstates.management.Systems.color;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TurretSensor extends DistanceCheckBase implements IsBallPresent{
    public RevColorSensorV3 sensor;
    private static double noBallDistance = 1;

    public TurretSensor(HardwareMap hardwareMap, TelemetryManager telemetryM) {
        Init(hardwareMap, telemetryM, "tCS", sensor);
    }

    public detectedLocation CheckIsBallPresent(){
        if (checkSensor(sensor, noBallDistance)){
            return detectedLocation.TURRET;
        } else {
            return detectedLocation.TurretCLEAR;
        }
    }
}
