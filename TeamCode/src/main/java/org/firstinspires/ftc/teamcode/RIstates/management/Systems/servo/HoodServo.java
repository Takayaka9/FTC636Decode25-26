package org.firstinspires.ftc.teamcode.RIstates.management.Systems.servo;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RIstates.management.Systems.servo.base.GamepadServoImplEx;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.servo.base.ServoImplExBase;

@Configurable
public class HoodServo extends ServoImplExBase implements GamepadServoImplEx {
    double start = 0.85;
    double end = 1;
    public HoodServo (HardwareMap hardwareMap) {
        super("hood", hardwareMap);
    }


}
