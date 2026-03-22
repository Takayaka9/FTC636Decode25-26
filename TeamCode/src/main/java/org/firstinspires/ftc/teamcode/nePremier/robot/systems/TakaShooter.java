package org.firstinspires.ftc.teamcode.nePremier.robot.systems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.nePremier.utils.commandUtils.BaseSubsystem;

public class TakaShooter extends BaseSubsystem {
    @Configurable
    private static class shooterTune {
        public static double Kp = 0.01;
        public static double Ki = 0.0005;
        public static double Kd = 0;
        public static double Kv = 0.00069;
        public static double Ks = 0.155;
        //public static double Kf = 0.00036;
        static double d1 = 36; static double r1 = 900;
        static double d2 = 50; static double r2 = 900;
        static double d3 = 75; static double r3 = 1000;
        static double d4 = 96; static double r4 = 1150;
        static double d5 = 108; static double r5 = 1400;
        static double d6 = 150; static double r6 = 1400;
        public static double brake = -0.3;
    }
    private final InterpLUT lut = new InterpLUT();
    private final DcMotorEx shooter1;
    private final DcMotorEx shooter2;

    public TakaShooter(HardwareMap hardwareMap) {
        super();
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setDirection(DcMotorEx.Direction.REVERSE);
        shooter1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lut.add(0, shooterTune.r1);
        lut.add(shooterTune.d1, shooterTune.r1);
        lut.add(shooterTune.d2, shooterTune.r2);
        lut.add(shooterTune.d3, shooterTune.r3);
        lut.add(shooterTune.d4, shooterTune.r4);
        lut.add(shooterTune.d5, shooterTune.r5);
        lut.add(shooterTune.d6, shooterTune.r6);
        lut.add(1000, shooterTune.r6);
        lut.createLUT();
    }



    //PID 1
    private final ElapsedTime pidTime1 = new ElapsedTime();
    private double integralSum1 = 0;
    private double lastError1 = 0;
    private double output1 = 0;
    private void update1(double target) {
        double error = target-(shooter1.getVelocity());
        double dt = pidTime1.seconds();
        if (dt < 0.0001) dt = 0.0001;
        integralSum1 += error* dt;
        double derivative = (error- lastError1)/ dt;
        lastError1 = error;

        pidTime1.reset();


        output1 = (error * shooterTune.Kp) + (derivative * shooterTune.Kd) + (integralSum1 * shooterTune.Ki) + shooterTune.Ks + (target * shooterTune.Kv);
        if (output1 < 0) {
            output1 = 0;
        }
        shooter1.setPower(output1);
    }

    //PID 2
    private final ElapsedTime pidTime2 = new ElapsedTime();
    private double lastError2 = 0;
    private double integralSum2 = 0;
    private double output2 = 0;
    private void update2(double target) {
        double error = target-(shooter2.getVelocity());
        double dt = pidTime2.seconds();
        if (dt < 0.0001) dt = 0.0001;
        integralSum2 += error* dt;
        double derivative = (error- lastError2)/ dt;
        lastError2 = error;

        pidTime2.reset();


        output2 = (error * shooterTune.Kp) + (derivative * shooterTune.Kd) + (integralSum2 * shooterTune.Ki) + shooterTune.Ks + (target * shooterTune.Kv);
        if (output1 < 0) {
            output1 = 0;
        }
        shooter2.setPower(output2);
    }



    /// MAIN RUN METHOD
    public void runForDistance(double distance){
        update1(getShooterTPS(distance));
        update2(getShooterTPS(distance));
    }
    /// SET POWER ZERO (DO NOT CALL RUN FOR DISTANCE WHEN USING THIS)
    public void stop(){
        shooter1.setPower(0);
        shooter2.setPower(0);
    }
    /// REVERSES THE SHOOTER
    public void reverse() {
        shooter1.setPower(-1);
        shooter2.setPower(-1);
    }
    /// SLIGHT REVERSE FOR BRAKING
    /// SETS POWER TO BRAKE POWER
    public void brake(){
        shooter1.setPower(shooterTune.brake);
        shooter2.setPower(shooterTune.brake);
    }
    /// TEST METHOD FOR PID
    public void test(double tps) {
        update2(tps);
        update1(tps);
    }
    /// CALC NEEDED TPS WITH LUT
    public int getShooterTPS(double targetDistance){
        return (int) Math.round(lut.get(targetDistance));
    }
    /// RETRIEVES AVERAGE VELOCITY OF BOTH SHOOTERS
    public int getAverageVelocity() {
        double averageVelocity = ((shooter1.getVelocity() + shooter2.getVelocity()) / 2);
        return (int) (Math.round(averageVelocity));
    }
    /// SHOOTER 1 TPS
    public int getShooter1tps() {
        return (int) Math.round(shooter1.getVelocity());
    }
    /// SHOOTER 2 TPS
    public int getShooter2tps() {
        return (int) Math.round(shooter2.getVelocity());
    }
    /// SET POWER FOR MOTOR ONE
    public double get1Power() {
        return output1;
    }
    /// SET POWER FOR MOTOR TWO
    public double get2Power() {
        return output2;
    }
    /// LAST ERROR
    public double getError(){
        //return getShooterTPS(distance) - shooter1.getVelocity();
        return lastError1;
    }

}
