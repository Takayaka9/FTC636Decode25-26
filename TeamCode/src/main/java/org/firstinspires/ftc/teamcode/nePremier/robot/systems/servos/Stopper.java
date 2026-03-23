package org.firstinspires.ftc.teamcode.nePremier.robot.systems.servos;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.nePremier.utils.servo.ServoImplexInterface;
import org.firstinspires.ftc.teamcode.nePremier.utils.servo.ServoImplExBase;

@Configurable
public class Stopper extends ServoImplExBase implements ServoImplexInterface {
    public double open = 1;
    public double closed = 0;
    public Stopper(HardwareMap hardwareMap) {
        super("stopper", hardwareMap);
    }
    public void close() {
        setPosition(closed);
    }
    public void open() {
        setPosition(open);
    }
}
