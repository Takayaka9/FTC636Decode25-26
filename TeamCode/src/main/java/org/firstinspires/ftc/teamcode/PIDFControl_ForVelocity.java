package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;

/*this is only for when we're doing PID(F) control over the VELOCITY of a motor,
just in case we ever do plus the code is like the same*/

@Configurable
public class PIDFControl_ForVelocity {
    ElapsedTime elapsedTime = new ElapsedTime();
    double integralSum = 0;
    double lastError;
    public double Kp;
    public double Ki;
    public double Kd;
    public double Kf;
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

        double output; // basically the same as the normal PIDControl
        output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (target * Kf);

        return Math.max(-1, Math.min(1, output)); //clamping so values do not exceed 1 or -1
    }
/* commented out for now, see if panels detects
    public void setValues(double p, double i, double d, double f){
        this.Kp = p;
        this.Ki = i;
        this.Kd = d;
        this.Kf = f;
    }*/
}
