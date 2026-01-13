package org.firstinspires.ftc.teamcode.RIstates.management.Systems.color;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Configurable
public class IntakeDistanceSensor extends DistanceCheckBase implements IsBallPresent{
    Rev2mDistanceSensor sensor;

    double noBallDistance = 90;
    public IntakeDistanceSensor(HardwareMap hardwareMap) {
        sensor = hardwareMap.get(Rev2mDistanceSensor.class, "iDS");
    }

    public detectedLocation CheckIsBallPresent(){
        if (checkDistanceSensor(sensor, noBallDistance)){
            return detectedLocation.INTAKE;
        } else {
            return detectedLocation.IntakeCLEAR;
        }
    }

    public double test() {
        return sensor.getDistance(DistanceUnit.MM);
    }


}
