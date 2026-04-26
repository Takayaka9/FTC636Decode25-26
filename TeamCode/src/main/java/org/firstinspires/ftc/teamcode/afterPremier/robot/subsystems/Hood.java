package org.firstinspires.ftc.teamcode.afterPremier.robot.subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.CommandBuilder;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.util.InterpLUT;
@Configurable
public class Hood {
    private final Servo h;
    private final InterpLUT lut = new InterpLUT();
    public Hood(HardwareMap hardwareMap){
        h = hardwareMap.get(Servo.class, "hood");
        lut.add(0, p1);
        lut.add(d1, p1);
        lut.add(d2, p2);
        lut.add(d3, p3);
        lut.add(d4, p4);
        lut.add(d5, p5);
        lut.add(d6, p6);
        lut.add(10000, p6);
        lut.createLUT();
    }
    static double d1 = 36; static double p1 = 0.99; //taka tuned
    static double d2 = 53.6; static double p2 = 0.88; //taka tuned
    static double d3 = 73.5; static double p3 = 0.85;//tuned
    static double d4 = 100; static double p4 = 0.85;//tuned
    static double d5 = 135.5; static double p5 = 0.85; //max
    static double d6 = 150; static double p6 = 0.85; //max
    public void angleHood(Pose current, Pose target) {
        double targetDistance = current.distanceFrom(target);
        double angle = lut.get(targetDistance);
        h.setPosition(angle);
    }
    public void down(){
        h.setPosition(1);
    }
}
