package org.firstinspires.ftc.teamcode.nePremier.utils.servo;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.nePremier.utils.commandUtils.BaseSubsystem;

public class ServoImplExBase extends BaseSubsystem implements ServoImplexInterface {
    final ServoImplEx servo;
    PwmRange range = null;
    private boolean pwmEnabled;

//    PwmConverter converter;

    public ServoImplExBase(String name, HardwareMap hardwareMap) {
        this(name, hardwareMap, true);
    }

    public ServoImplExBase(String name, HardwareMap hardwareMap, boolean enablePwmOnInit) {
        super();
        servo = hardwareMap.get(ServoImplEx.class, name);
        pwmEnabled = enablePwmOnInit;
//        converter = new PwmConverter();
        if (enablePwmOnInit) {
            servo.setPwmEnable();
        }
    }

//    @Deprecated
//    public void setRange(double start, double end) {
//        range = new PwmControl.PwmRange(converter.convert(start), converter.convert(end));
//        servo.setPwmRange(range);
//    }

    public void setPosition(double position) {
        if (!pwmEnabled) {
            servo.setPwmEnable();
            pwmEnabled = true;
        }
        servo.setPosition(position);
    }

    public double getPosition(){
        return servo.getPosition();
    }

    public void togglePosition(boolean open) {
        if (!pwmEnabled) {
            servo.setPwmEnable();
            pwmEnabled = true;
        }
        if (open) {
            servo.setPosition(0);
        } else {
            servo.setPosition(1);
        }
    }

}
