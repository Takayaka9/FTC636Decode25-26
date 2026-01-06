package org.firstinspires.ftc.teamcode.RIstates.management.handlers.fsm.states.controllers.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Belt {
    HardwareMap hardwareMap;
    CRServo belt;
    public Belt(HardwareMap hardwareMap, String name) {
        this.hardwareMap = hardwareMap;
        this.belt = hardwareMap.get(CRServo.class, name);
    }

    public static int beltPower = 1;

    public void run() {
        belt.setPower(beltPower);
    }

    public void stop() {
        belt.setPower(0);
    }

    public void reverse() {
        belt.setPower(-1);
    }

}
