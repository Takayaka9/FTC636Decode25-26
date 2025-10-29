package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

/*
This is an abstract class for PIDControl over the position of a motor such
as vertical slides to minimize error. To use it, make an instance of the class
and pass through the values of Kp, Ki and Kd. Those values need to be tuned.
*/

public class PIDControl {
    ElapsedTime elapsedTime = new ElapsedTime();
    double integralSum = 0;
    double lastError;
    double Kp;
    double Ki;
    double Kd;
    public PIDControl(double Kp, double Ki, double Kd){
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public double update(double target, double state){
        double error = target-state;
        integralSum += error* elapsedTime.seconds();
        double derivative = (error- lastError)/ elapsedTime.seconds();
        lastError = error;

        elapsedTime.reset();

        double output; // define separately for a little more neatness (and less warnings)
        output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }
}
