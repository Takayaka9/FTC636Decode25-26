package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

/*this is only for when we're doing PID(F) control over the VELOCITY of a motor,
just in case we ever do plus the code is like the same*/

public class PIDFControl_ForVelocity {
    ElapsedTime elapsedTime = new ElapsedTime();
    double integralSum = 0;
    double lastError;
    double Kp;
    double Ki;
    double Kd;
    double Kf;
    public PIDFControl_ForVelocity(double Kp, double Ki, double Kd, double Kf){
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kf = Kf; // tune Kf first
    }

    public double update(double target, double state){
        double error = target-state;
        integralSum += error* elapsedTime.seconds();
        double derivative = (error- lastError)/ elapsedTime.seconds();
        lastError = error;

        elapsedTime.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (target * Kf);
        return output;
    }
}
