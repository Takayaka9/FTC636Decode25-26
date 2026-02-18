package org.firstinspires.ftc.teamcode.zRIstates.management.Systems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Intake {
    DcMotorEx intake;
    public Intake(HardwareMap hardwareMap, String name) {
        intake = hardwareMap.get(DcMotorEx.class, name);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public static double intakePower = 1;
    public void run(){
        intake.setPower(intakePower);
    }
    public static double shootPower = 1;
    public void shootRun(){
        intake.setPower(shootPower);
    }
    double reverseIntakePower = -1;
    public void reverse(){
        intake.setPower(reverseIntakePower);
    }

    public void stop(){
        intake.setPower(0);
    }

}
