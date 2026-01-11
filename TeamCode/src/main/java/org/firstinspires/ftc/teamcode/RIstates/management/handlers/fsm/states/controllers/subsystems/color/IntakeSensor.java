package org.firstinspires.ftc.teamcode.RIstates.management.handlers.fsm.states.controllers.subsystems.color;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;


@Configurable
public class IntakeSensor extends DistanceCheckBase implements IsBallPresent{
    public RevColorSensorV3 sensor;
    private static double noBallDistance = 1;

    public IntakeSensor(HardwareMap hardwareMap, TelemetryManager telemetryM) {
        Init(hardwareMap, telemetryM, "iCS", sensor);
    }

    public detectedLocation CheckIsBallPresent(){
        if (checkSensor(sensor, noBallDistance)){
        return detectedLocation.INTAKE;
        } else {
            return detectedLocation.INTAKECLEAR;
        }
    }
}
