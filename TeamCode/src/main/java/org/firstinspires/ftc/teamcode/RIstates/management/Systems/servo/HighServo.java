package org.firstinspires.ftc.teamcode.RIstates.management.Systems.servo;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class HighServo //implements GamepadServoImplEx{
{

    ServoImplEx servo = null;
    public HighServo(HardwareMap hardwareMap) {
        servo = hardwareMap.get(ServoImplEx.class, "servo");
    }


    public void setPosition(double position) {
        servo.setPosition(position);
    }

    public double getPosition() {
        return servo.getPosition();
    }

    public void togglePosition(boolean open) {

    }

}
