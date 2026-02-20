package org.firstinspires.ftc.teamcode.zRIstates.management.Systems.color;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.sensors.DistanceCheckBase;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.sensors.IsBallPresent;


@Configurable
public class IntakeSensor extends DistanceCheckBase implements IsBallPresent {
    public RevColorSensorV3 sensor;
    private static double noBallDistance = 57.5;

    public IntakeSensor(HardwareMap hardwareMap) {
//        Init(hardwareMap, telemetryM, "iCS", sensor);
        sensor = hardwareMap.get(RevColorSensorV3.class, "iCS");
        sensor.setGain(1);
    }


    @Override
    public detectedLocation CheckIsBallPresent(){
        if (checkSensor(sensor, noBallDistance)){
        return detectedLocation.INTAKE;
        } else {
            return detectedLocation.IntakeCLEAR;
        }
    }

    @Override
    public double test() {
        return sensor.getDistance(DistanceUnit.MM);
    }
}
