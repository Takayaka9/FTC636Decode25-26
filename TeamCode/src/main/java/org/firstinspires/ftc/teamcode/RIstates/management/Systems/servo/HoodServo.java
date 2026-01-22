package org.firstinspires.ftc.teamcode.RIstates.management.Systems.servo;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RIstates.management.Systems.servo.base.GamepadServo;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.servo.base.ServoImplExBase;

@Configurable
public class HoodServo extends ServoImplExBase implements GamepadServo {
    double start = 0.2;
    double end = .5;
    public HoodServo (HardwareMap hardwareMap) {
        super("hood", hardwareMap);
        super.setRange(start, end);
    }


}
