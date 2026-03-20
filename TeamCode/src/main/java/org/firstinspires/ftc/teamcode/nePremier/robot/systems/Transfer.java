package org.firstinspires.ftc.teamcode.nePremier.robot.systems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.nePremier.utils.commandUtils.BaseSubsystem;

public class Transfer extends BaseSubsystem {
    @Configurable
    static class TransferPower {
        public static double power = 1;
        public static double reversePower = -1;
    }
    private final DcMotorEx transfer;
    public Transfer(HardwareMap hardwareMap) {
        super();
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        transfer.setDirection(DcMotorEx.Direction.REVERSE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void run() {
        transfer.setPower(TransferPower.power);
    }
    public void reverse() {
        transfer.setPower(TransferPower.reversePower);
    }

    public void stop() {
        transfer.setPower(0);
    }

}
