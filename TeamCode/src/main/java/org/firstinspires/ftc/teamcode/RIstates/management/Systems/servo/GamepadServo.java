package org.firstinspires.ftc.teamcode.RIstates.management.Systems.servo;

public interface GamepadServo {
    void setPosition(double position);
    double getPosition();
    void togglePosition(boolean open);
    void setRange(double start, double end);

}
