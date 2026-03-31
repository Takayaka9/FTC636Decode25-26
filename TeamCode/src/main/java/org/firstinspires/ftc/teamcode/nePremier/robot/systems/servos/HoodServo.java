package org.firstinspires.ftc.teamcode.nePremier.robot.systems.servos;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.nePremier.utils.servo.ServoImplexInterface;
import org.firstinspires.ftc.teamcode.nePremier.utils.servo.ServoImplExBase;

@Configurable
public class HoodServo extends ServoImplExBase implements ServoImplexInterface {
    private final InterpLUT lut = new InterpLUT();
    public HoodServo (HardwareMap hardwareMap) {
        super("hood", hardwareMap);
        lut.add(0, p1);
        lut.add(d1, p1);
        lut.add(d2, p2);
        lut.add(d3, p3);
        lut.add(d4, p4);
        lut.add(d5, p5);
        lut.add(d6, p6);
        lut.add(1000, p6);
        lut.createLUT();
    }

    static double d1 = 36; static double p1 = 0.99; //taka tuned
    static double d2 = 53.6; static double p2 = 0.88; //taka tuned
    static double d3 = 73.5; static double p3 = 0.85;//tuned
    static double d4 = 100; static double p4 = 0.85;//tuned
    static double d5 = 135.5; static double p5 = 0.85; //max
    static double d6 = 150; static double p6 = 0.85; //max

    public void angleHood(double targetDistance) {
        double angle = lut.get(targetDistance);
        setPosition(angle);
    }

    public void passive(){
        setPosition(1);
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
