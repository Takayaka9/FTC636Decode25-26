package org.firstinspires.ftc.teamcode.RIstates.management.Systems.servo;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class GateServo extends ServoImplExBase implements GamepadServo {
    static double start = 0;
    static double end = .6;
    public GateServo (HardwareMap hardwareMap) {
        super("hood", hardwareMap);
    }


}
