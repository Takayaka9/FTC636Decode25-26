package org.firstinspires.ftc.teamcode.utils.pwm;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class PwmConverter {
    public double convert(double input) {
        int multiplied = (int) input * 1000;
        int added = multiplied + 1000;
        double output = 0;
        if (added > 2000) {
            output = 2000;
        } else if (added < 1000) {
            output = 1000;
        } else if (added > 1000 && added < 2000){
            output = added;
        }

        return output;
    }
}
