package org.firstinspires.ftc.teamcode.NewEnglands.robot.systems.servos;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.NewEnglands.utils.servo.GamepadServoImplEx;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.servo.ServoImplExBase;

@Configurable
public class HoodServo extends ServoImplExBase implements GamepadServoImplEx {
    double start = 0.85;
    double end = 1;
    private final InterpLUT lut = new InterpLUT();
    public HoodServo (HardwareMap hardwareMap) {
        super(maps,"hood", hardwareMap);
        lut.add(0, 1);
        lut.add(d1, p1);
        lut.add(d2, p2);
        lut.add(d3, p3);
        lut.add(d4, p4);
        lut.add(d5, p5);
        lut.add(d6, p6);
        lut.add(1000, p6);
        lut.createLUT();
    }

    static double d1 = 36; static double p1 = 0.96;
    static double d2 = 50; static double p2 = 0.95;
    static double d3 = 75; static double p3 = 0.92;
    static double d4 = 96; static double p4 = 0.85;
    static double d5 = 108; static double p5 = 0.85;
    static double d6 = 150; static double p6 = 0.85;

    public double angle;
    public void angleHood(double targetDistance) {
        angle = lut.get(targetDistance);
        setPosition(angle);
    }

    public static double passive = 1;
    public void passive(){
        setPosition(passive);
    }

    public void increment(boolean positive){
        if(positive){
            setPosition(getPosition() + 0.03);
        }
        else{
            setPosition(getPosition() - 0.03);
        }
    }


}
