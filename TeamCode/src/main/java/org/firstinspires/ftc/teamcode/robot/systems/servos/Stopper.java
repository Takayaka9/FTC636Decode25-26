package org.firstinspires.ftc.teamcode.robot.systems.servos;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.commandUtils.CommandLoop;
import org.firstinspires.ftc.teamcode.utils.servo.GamepadServoImplEx;
import org.firstinspires.ftc.teamcode.utils.servo.ServoImplExBase;

@Configurable
public class Stopper extends ServoImplExBase implements GamepadServoImplEx {
    static double start = 0;
    static double end = .6;
    public double open = 0;
    public double closed = .6;
    public Stopper(CommandLoop maps, HardwareMap hardwareMap) {
        super(maps,"stopper", hardwareMap);
    }
    public void close() {
        setPosition(closed);
    }
    public void open() {
        setPosition(open);
    }
}
