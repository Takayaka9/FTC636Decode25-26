package org.firstinspires.ftc.teamcode.RIstates.management.Systems.Belt;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Belt {
    HardwareMap hardwareMap;
    //CRServo belt;
    DcMotorEx belt;
    public Belt(HardwareMap hardwareMap, String name) {
        this.hardwareMap = hardwareMap;
        //this.belt = hardwareMap.get(CRServo.class, name);
        this.belt = hardwareMap.get(DcMotorEx.class, name);
    }

    public static double beltPower = 0.5;

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
