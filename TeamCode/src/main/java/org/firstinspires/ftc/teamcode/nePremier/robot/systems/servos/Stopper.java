package org.firstinspires.ftc.teamcode.nePremier.robot.systems.servos;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.nePremier.utils.commandUtils.BaseSubsystem;
import org.firstinspires.ftc.teamcode.nePremier.utils.servo.ServoImplexInterface;
import org.firstinspires.ftc.teamcode.nePremier.utils.servo.ServoImplExBase;

@Configurable
public class Stopper extends BaseSubsystem {
    public static double open = 0.831;
    public static double closed = 1;
    private final Servo s;
    public Stopper(HardwareMap hardwareMap){
        s = hardwareMap.get(Servo.class, "stopper");
    }
    public void setPosition(double position) {
        s.setPosition(position);
    }
    public void close() {
        s.setPosition(closed);
    }
    public void open() {
        s.setPosition(open);
    }
}
