package org.firstinspires.ftc.teamcode.NewEnglands.robot.systems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.BaseSubsystem;

public class TakaShooter extends BaseSubsystem {
    @Configurable
    public static class shooterTune {
        public static double Kp = 0.3;
        public static double Ki = 0;
        public static double Kd = 0;
        public static double Kv = 0;
        public static double Ks = 0;
        //public static double Kf = 0.00036;
        static double d1 = 36; static double r1 = 900;
        static double d2 = 50; static double r2 = 900;
        static double d3 = 75; static double r3 = 1000;
        static double d4 = 96; static double r4 = 1150;
        static double d5 = 108; static double r5 = 1400;
        static double d6 = 150; static double r6 = 1400;
        public static double brake = -0.3;
    }
    TelemetryManager telemetryM;
    private final InterpLUT lut = new InterpLUT();
    DcMotorEx shooter1;
    DcMotorEx shooter2;
    public static int targetTPS = 0;

    public TakaShooter(HardwareMap hardwareMap) {
        super();
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter1.setDirection(DcMotorEx.Direction.REVERSE);
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
    private ElapsedTime pidTime1 = new ElapsedTime();
    private double integralSum1;
    private double lastError1;
    private double output1;
    public void updateRight(double target) {
        double error = target-(shooter1.getVelocity());
        double dt = pidTime1.seconds();
        if (dt < 0.0001) dt = 0.0001;
        integralSum1 += error* dt;
        double derivative = (error- lastError1)/ dt;
        lastError1 = error;

        pidTime1.reset();


        output1 = (error * shooterTune.Kp) + (derivative * shooterTune.Kd) + (integralSum1 * shooterTune.Ki) + shooterTune.Ks + (target * shooterTune.Kv);

        shooter1.setPower(output1);
    }

    //PID 2
    ElapsedTime pidTime2 = new ElapsedTime();
    double lastError2;
    double integralSum2;
    public double output2 = 0;
    public void updateLeft(double target) {
        double error = target-(shooter2.getVelocity());
        double dt = pidTime2.seconds();
        if (dt < 0.0001) dt = 0.0001;
        integralSum2 += error* dt;
        double derivative = (error- lastError2)/ dt;
        lastError2 = error;

        pidTime2.reset();


        output2 = (error * shooterTune.Kp) + (derivative * shooterTune.Kd) + (integralSum2 * shooterTune.Ki) + shooterTune.Ks + (target * shooterTune.Kv);

        shooter2.setPower(output2);
    }



    //utils
    public void runForDistance(double distance){
        updateRight(getShooterTPS(distance));
        updateLeft(getShooterTPS(distance));
    }
    public void stop(){
        shooter1.setPower(0);
        shooter2.setPower(0);
    }
    public void reverse() {
        shooter1.setPower(-1);
        shooter2.setPower(-1);
    }
    public void brake(){
        shooter1.setPower(shooterTune.brake);
        shooter2.setPower(shooterTune.brake);
    }
    public void test(double tps) {
        updateLeft(tps);
        updateRight(tps);
    }
    public int getShooterTPS(double targetDistance){
        int calcTPS = (int) Math.round(lut.get(targetDistance));
        //telemetryM.addData("Calculated TPS", calcTPS);
        targetTPS = calcTPS;
        return calcTPS;
    }
    public int averageVelocity () {
        double averageVelocity = ((shooter1.getVelocity() + shooter2.getVelocity()) / 2);
        return (int) (Math.round(averageVelocity));
    }
    public double getError(){
        //return getShooterTPS(distance) - shooter1.getVelocity();
        return lastError1;
    }

}
