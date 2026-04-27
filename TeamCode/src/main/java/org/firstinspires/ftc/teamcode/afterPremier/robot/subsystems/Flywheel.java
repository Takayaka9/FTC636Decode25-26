package org.firstinspires.ftc.teamcode.afterPremier.robot.subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.CommandBuilder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.nePremier.robot.systems.TakaShooter;

@Configurable
public class Flywheel {
    private final DcMotorEx f1;
    private final DcMotorEx f2;
    private final InterpLUT lut = new InterpLUT();
    public enum ShootState{
        ON, OFF
    }
    ShootState state;
    public Flywheel(HardwareMap hardwareMap){
        f1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        f2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        f1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        f2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        f2.setDirection(DcMotorEx.Direction.REVERSE);
        f1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        f2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lut.add(0, r1);
        lut.add(d1, r1);
        lut.add(d2, r2);
        lut.add(d3, r3);
        lut.add(d4, r4);
        lut.add(d5, r5);
        lut.add(d6, r6);
        lut.add(1000, r6);
        lut.createLUT();
    }
    private void bangbang(double target, DcMotorEx m){
        if(Math.abs(target - f2.getVelocity()) < 40){
            m.setPower(0);
        }
        else if(target > f2.getVelocity()){
            m.setPower(1);
        }
    }
    private void pid(double target, DcMotorEx m){
        double vel = f2.getVelocity();
        double error = target - vel;
        double feedForward = Ks + (target * Kv);
        double output = error* Kp + feedForward;

        m.setPower(output);
    }
    public void run(Pose current, Pose goal){
        double target = lut.get(current.distanceFrom(goal));
        bangbang(target, f1);
        bangbang(target, f2);
    }
    public void stop(){
        f1.setPower(0);
        f2.setPower(0);
    }
    public static double Kp = 0.8;
    public static double Kd = 0;
    public static double Kv = 0.00049;
    public static double Ks = 0.17;
    public static double minActiveTps = 900;
    public static double maxError = 3;
    //public static double Kf = 0.00036;
    static double d1 = 36; static double r1 = 850;//tuned
    static double d2 = 53.6; static double r2 = 1000;//tuned
    static double d3 = 73.5; static double r3 = 1075;//tuned
    static double d4 = 100; static double r4 = 1125;//tuned
    static double d5 = 135.5; static double r5 = 1350;//tuned
    static double d6 = 150; static double r6 = 1360;//tuned
    public static double brake = -0.3;
}
