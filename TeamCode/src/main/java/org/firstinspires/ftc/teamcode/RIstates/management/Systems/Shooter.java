package org.firstinspires.ftc.teamcode.RIstates.management.Systems;

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
    public final DcMotorEx flyRight;
    public final DcMotorEx flyLeft;
    private ElapsedTime pidTime = new ElapsedTime();
    //TODO: decide lut or no lut

    static double d1 = 36; static double r1 = 1100;
    static double d2 = 50; static double r2 = 1400;
    static double d3 = 75; static double r3 = 1500;
    static double d4 = 96; static double r4 = 1850;
    static double d5 = 108; static double r5 = 2799;
    static double d6 = 150; static double r6 = 2800;

    public Shooter(HardwareMap hardwareMap, String rightName, String leftName){
        flyRight = hardwareMap.get(DcMotorEx.class, rightName);
        flyLeft = hardwareMap.get(DcMotorEx.class, leftName);
        flyRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyRight.setDirection(DcMotorEx.Direction.REVERSE);
        flyRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flyLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
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
        flyRight.setPower(0);
        flyLeft.setPower(0);
    }


    public static double Kp = 0.3;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double Kf = 0.00036;
    //0.000357 should be the max kf value
    //TODO: either make a pidf class (good idea) or just fix ts. either way use chatgpt.
    private double integralSum;
    private double lastError;
    public double outputRight; // basically the same as the normal PIDControl
    public void updateRight(double target){
        double error = target-(flyRight.getVelocity());
        double dt = pidTime.seconds();
        if (dt < 0.0001) dt = 0.0001;
        integralSum += error* dt;
        double derivative = (error- lastError)/ dt;
        lastError = error;

        pidTime.reset();


        outputRight = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (target * Kf);

        flyRight.setPower(outputRight);
        //flyLeft.setPower(output); //in case we switch back to one pid
    }

    ElapsedTime pidLeft = new ElapsedTime();
    double lastErrorLeft;
    double integralSumLeft;
    public double outputLeft = 0; // basically the same as the normal PIDControl
    public void updateLeft(double target){
        double error = target-(flyLeft.getVelocity());
        double dt = pidLeft.seconds();
        if (dt < 0.0001) dt = 0.0001;
        integralSumLeft += error* dt;
        double derivative = (error- lastErrorLeft)/ dt;
        lastErrorLeft = error;

        pidLeft.reset();


        outputLeft = (error * Kp) + (derivative * Kd) + (integralSumLeft * Ki) + (target * Kf);

        flyLeft.setPower(outputLeft);
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
        flyRight.setPower(-1);
        flyLeft.setPower(-1);
    }
    public int averageVelocity () {
        double averageVelocity = ((flyRight.getVelocity() + flyLeft.getVelocity()) / 2);
        return (int) (Math.round(averageVelocity));
    }
}
