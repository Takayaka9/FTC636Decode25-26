package org.firstinspires.ftc.teamcode.RIstates.management.handlers.fsm.states.controllers.subsystems;

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

    double d1 = 0; int r1 = 0;
    double d2 = 0; int r2 = 0;
    double d3 = 0; int r3 = 0;
    double d4 = 0; int r4 = 0;
    double d5 = 0; int r5 = 0;
    double d6 = 0; int r6 = 0;
    public Shooter(HardwareMap hardwareMap, String rightName, String leftName){
        flyRight = hardwareMap.get(DcMotorEx.class, rightName);
        flyLeft = hardwareMap.get(DcMotorEx.class, leftName);
        flyRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyRight.setDirection(DcMotorEx.Direction.REVERSE);
        flyRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flyLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lut.add(d1, r1);
        lut.add(d2, r2);
        lut.add(d3, r3);
        lut.add(d4, r4);
        lut.add(d5, r5);
        lut.add(d6, r6);
        lut.createLUT();
    }

    public int getShooterTPS(double targetDistance){

        int calcTPS = (int) Math.round(lut.get(targetDistance));
        //telemetryM.addData("Calculated TPS", calcTPS);

        return calcTPS;
    }

    public void stop(){
        flyRight.setPower(0);
        flyLeft.setPower(0);
    }


    public static double Kp = 0.004;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double Kf = 0.006;



    private double integralSum;
    private double lastError;
    public double outputRight; // basically the same as the normal PIDControl
    public void updateRight(double reference){
        double error = reference-(flyRight.getVelocity());
        double dt = pidTime.seconds();
        if (dt < 0.0001) dt = 0.0001;
        integralSum += error* dt;
        double derivative = (error- lastError)/ dt;
        lastError = error;

        pidTime.reset();


        outputRight = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (reference * Kf);

        flyRight.setPower(outputRight);
        //flyLeft.setPower(output); //in case we switch back to one pid
    }

    ElapsedTime pidLeft = new ElapsedTime();
    double lastErrorLeft;
    double integralSumLeft;
    public double outputLeft = 0; // basically the same as the normal PIDControl
    public void updateLeft(double reference){
        double error = reference-(flyLeft.getVelocity());
        double dt = pidLeft.seconds();
        if (dt < 0.0001) dt = 0.0001;
        integralSumLeft += error* dt;
        double derivative = (error- lastErrorLeft)/ dt;
        lastErrorLeft = error;

        pidLeft.reset();


        outputLeft = (error * Kp) + (derivative * Kd) + (integralSumLeft * Ki) + (reference * Kf);

        flyLeft.setPower(outputLeft);
    }

    public void shoot(double distance){
        updateRight(getShooterTPS(distance));
        updateLeft(getShooterTPS(distance));
    }

    public void reverse() {
        flyRight.setPower(-1);
        flyLeft.setPower(-1);
    }
}
