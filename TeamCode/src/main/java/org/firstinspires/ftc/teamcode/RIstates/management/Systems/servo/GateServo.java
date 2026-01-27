package org.firstinspires.ftc.teamcode.RIstates.management.Systems.servo;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RIstates.management.Systems.servo.base.GamepadServoImplEx;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.servo.base.ServoImplExBase;

@Configurable
public class GateServo extends ServoImplExBase implements GamepadServoImplEx {
    static double start = 0;
    static double end = .6;
    public GateServo (HardwareMap hardwareMap) {
        super("hood", hardwareMap);
    }


}
