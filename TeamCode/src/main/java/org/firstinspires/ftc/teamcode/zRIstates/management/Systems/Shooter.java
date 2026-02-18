package org.firstinspires.ftc.teamcode.zRIstates.management.Systems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.util.InterpLUT;

@Configurable
public class Shooter {
    TelemetryManager telemetryM;
    private final InterpLUT lut = new InterpLUT();
    public final DcMotorEx shooter1;
    public final DcMotorEx shooter2;
    private ElapsedTime pidTime = new ElapsedTime();

    static double d1 = 36; static double r1 = 900;
    static double d2 = 50; static double r2 = 900;
    static double d3 = 75; static double r3 = 1000;
    static double d4 = 96; static double r4 = 1150;
    static double d5 = 108; static double r5 = 1400;
    static double d6 = 150; static double r6 = 1400;

    public Shooter(HardwareMap hardwareMap, String rightName, String leftName){
        shooter1 = hardwareMap.get(DcMotorEx.class, rightName);
        shooter2 = hardwareMap.get(DcMotorEx.class, leftName);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter1.setDirection(DcMotorEx.Direction.REVERSE);
        shooter1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
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
    public static int targetTPS = 0;
    public int getShooterTPS(double targetDistance){
        int calcTPS = (int) Math.round(lut.get(targetDistance));
        //telemetryM.addData("Calculated TPS", calcTPS);
        targetTPS = calcTPS;
        return calcTPS;
    }

    public void stop(){
        shooter1.setPower(0);
        shooter2.setPower(0);
    }


    public static double Kp = 0.3;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double Kf = 0.00036;
    //0.000357 should be the max kf value
    private double integralSum;
    private double lastError;
    public double outputRight; // basically the same as the normal PIDControl
    public void updateRight(double target){
        double error = target-(shooter1.getVelocity());
        double dt = pidTime.seconds();
        if (dt < 0.0001) dt = 0.0001;
        integralSum += error* dt;
        double derivative = (error- lastError)/ dt;
        lastError = error;

        pidTime.reset();


        outputRight = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (target * Kf);

        shooter1.setPower(outputRight);
        //shooter2.setPower(output); //in case we switch back to one pid
    }

    ElapsedTime pidLeft = new ElapsedTime();
    double lastErrorLeft;
    double integralSumLeft;
    public double outputLeft = 0; // basically the same as the normal PIDControl
    public void updateLeft(double target){
        double error = target-(shooter2.getVelocity());
        double dt = pidLeft.seconds();
        if (dt < 0.0001) dt = 0.0001;
        integralSumLeft += error* dt;
        double derivative = (error- lastErrorLeft)/ dt;
        lastErrorLeft = error;

        pidLeft.reset();


        outputLeft = (error * Kp) + (derivative * Kd) + (integralSumLeft * Ki) + (target * Kf);

        shooter2.setPower(outputLeft);
    }

    public double getError(){
        //return getShooterTPS(distance) - shooter1.getVelocity();
        return lastError;
    }

    public void shoot(double distance){
        updateRight(getShooterTPS(distance));
        updateLeft(getShooterTPS(distance));
    }
    public void test(double tps) {
        updateLeft(tps);
        updateRight(tps);
    }

    public void reverse() {
        shooter1.setPower(-1);
        shooter2.setPower(-1);
    }
    public static double brake = -0.3;
    public void brake(){
        shooter1.setPower(brake);
        shooter2.setPower(brake);
    }
    public int averageVelocity () {
        double averageVelocity = ((shooter1.getVelocity() + shooter2.getVelocity()) / 2);
        return (int) (Math.round(averageVelocity));
    }
}
