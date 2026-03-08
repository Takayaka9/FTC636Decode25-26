package org.firstinspires.ftc.teamcode.NewEnglands.utils.servo;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.BaseSubsystem;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.CommandLoop;

@Configurable
public abstract class CRServoBase extends BaseSubsystem {
    HardwareMap hardwareMap;
    CRServo belt;
//    DcMotorEx belt;
    public CRServoBase(CommandLoop maps, HardwareMap hardwareMap, String name) {
        super();
        this.hardwareMap = hardwareMap;
        this.belt = hardwareMap.get(CRServo.class, name);
//        this.belt = hardwareMap.get(DcMotorEx.class, name);
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
