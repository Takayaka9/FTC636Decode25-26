package org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.states.controllers.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Intake {
    DcMotorEx intake;
    public Intake(HardwareMap hardwareMap, String name) {
        intake = hardwareMap.get(DcMotorEx.class, name);
    }

    double intakePower = 1;
    public void run(){
        intake.setPower(intakePower);
    }
    double reverseIntakePower = -1;
    public void reverse(){
        intake.setPower(reverseIntakePower);
    }

    public void stop(){
        intake.setPower(0);
    }

}
