package org.firstinspires.ftc.teamcode.RIstates.management.Systems.servo.base;

public interface GamepadServoImplEx {
    void setPosition(double position);
    double getPosition();
    void togglePosition(boolean open);
    void setRange(double start, double end);

}
