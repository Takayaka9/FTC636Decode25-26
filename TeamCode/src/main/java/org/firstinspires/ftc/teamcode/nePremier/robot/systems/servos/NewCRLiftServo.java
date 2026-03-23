package org.firstinspires.ftc.teamcode.nePremier.robot.systems.servos;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.nePremier.utils.commandUtils.BaseSubsystem;

public class NewCRLiftServo extends BaseSubsystem {
    private final CRServo servo1;
    private final CRServo servo2;
    public NewCRLiftServo(HardwareMap hardwareMap) {
        super();
        servo1 = hardwareMap.get(CRServo.class, "lift1");
        servo2 = hardwareMap.get(CRServo.class , "lift2");
        //servo1.setDirection(DcMotorSimple.Direction.REVERSE);
        //servo2.setDirection(DcMotorSimple.Direction.FORWARD);
//        servo1.setPower(0);
//        servo2.setPower(0);
    }

    public void forward() {
        servo1.setPower(1);
        servo2.setPower(-1);
    }

    public void backward() {
        servo1.setPower(-1);
        servo2.setPower(1);
    }

    public void stop() {
        servo1.setPower(0);
        servo2.setPower(0);
    }
}
