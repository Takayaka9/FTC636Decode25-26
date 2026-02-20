package org.firstinspires.ftc.teamcode.NewEnglands.utils.servo;

public interface GamepadServoImplEx {
    void setPosition(double position);
    double getPosition();
    void togglePosition(boolean open);
    void setRange(double start, double end);

}
