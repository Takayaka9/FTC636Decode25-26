package org.firstinspires.ftc.teamcode.states.Management.Handlers.FSM.states.Controllers.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.util.InterpLUT;

@Configurable
public class Shooter {
    TelemetryManager telemetryM;
    private final InterpLUT lut = new InterpLUT();
    private final DcMotorEx flyRight;
    private final DcMotorEx flyLeft;
    private ElapsedTime pidTime = new ElapsedTime();
    private double integralSum;
    private double lastError;
    public static double Kp = 0.004;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double Kf = 0.006;
    double d1 = 0; int r1 = 0;
    double d2 = 0; int r2 = 0;
    double d3 = 0; int r3 = 0;
    double d4 = 0; int r4 = 0;
    double d5 = 0; int r5 = 0;
    double d6 = 0; int r6 = 0;
    public Shooter(HardwareMap hardwareMap, String rightName, String leftName){
        flyRight = hardwareMap.get(DcMotorEx.class, rightName);
        flyLeft = hardwareMap.get(DcMotorEx.class, leftName);
        lut.add(d1, r1);
        lut.add(d2, r2);
        lut.add(d3, r3);
        lut.add(d4, r4);
        lut.add(d5, r5);
        lut.add(d6, r6);
        lut.createLUT();
    }

    public int shooterRPM(double targetDistance){

        int calcRPM = (int) Math.round(lut.get(targetDistance));
        //telemetryM.addData("Calculated RPM", calcRPM);

        return calcRPM;
    }

    public void stop(){
        flyRight.setPower(0);
        flyLeft.setPower(0);
    }

    public void updateRight(double reference){
        double error = reference-(flyRight.getVelocity());
        double dt = pidTime.seconds();
        if (dt < 0.0001) dt = 0.0001;
        integralSum += error* dt;
        double derivative = (error- lastError)/ dt;
        lastError = error;

        pidTime.reset();

        double output; // basically the same as the normal PIDControl
        output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (reference * Kf);

        //flyRight.setPower(output);
        //flyLeft.setPower(output); //in case we switch back to one pid
    }
    ElapsedTime pidLeft = new ElapsedTime();
    double lastErrorLeft;
    double integralSumLeft;
    public void updateLeft(double reference){
        double error = reference-(flyLeft.getVelocity());
        double dt = pidLeft.seconds();
        if (dt < 0.0001) dt = 0.0001;
        integralSumLeft += error* dt;
        double derivative = (error- lastErrorLeft)/ dt;
        lastErrorLeft = error;

        pidLeft.reset();

        double output; // basically the same as the normal PIDControl
        output = (error * Kp) + (derivative * Kd) + (integralSumLeft * Ki) + (reference * Kf);

        //flyLeft.setPower(output);
    }

    public void shoot(double distance){
        updateRight(shooterRPM(distance));
        updateLeft(shooterRPM(distance));
    }

    public void reverse() {
        flyRight.setPower(-1);
        flyLeft.setPower(-1);
    }
}
