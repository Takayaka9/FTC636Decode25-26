package org.firstinspires.ftc.teamcode.NewEnglands.utils.servo;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.BaseSubsystem;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.CommandLoop;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.pwm.PwmConverter;

public class ServoImplExBase extends BaseSubsystem implements GamepadServoImplEx {
    ServoImplEx servo;
    PwmRange range;

    PwmConverter converter;

    public ServoImplExBase(String name, HardwareMap hardwareMap) {
        super();
        servo = hardwareMap.get(ServoImplEx.class, name);
        converter = new PwmConverter();
        servo.setPwmEnable();
    }

    public void setRange(double start, double end) {
        range = new PwmControl.PwmRange(converter.convert(start), converter.convert(end));
        servo.setPwmRange(range);
    }

    public void setPosition(double position) {
        servo.setPosition(position);
    }

    public double getPosition(){
        return servo.getPosition();
    }

    public void togglePosition(boolean open) {
        if (open) {
            servo.setPosition(0);
        } else {
            servo.setPosition(1);
        }
    }

}
