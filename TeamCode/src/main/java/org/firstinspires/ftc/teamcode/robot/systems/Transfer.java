package org.firstinspires.ftc.teamcode.robot.systems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.utils.commandUtils.BaseSubsystem;
import org.firstinspires.ftc.teamcode.utils.commandUtils.CommandLoop;
import org.firstinspires.ftc.teamcode.utils.servo.ServoImplExBase;

public class Transfer extends ServoImplExBase {
    @Configurable
    static class TransferPower {
        public static double power = 1;
        public static double reversePower = -1;
    }
    private DcMotorEx transfer;
    public Transfer(CommandLoop maps, HardwareMap hardwareMap) {
        super(maps, "transfer", hardwareMap);
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
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
