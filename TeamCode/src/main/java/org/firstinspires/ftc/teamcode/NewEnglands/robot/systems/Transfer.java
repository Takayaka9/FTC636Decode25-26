package org.firstinspires.ftc.teamcode.NewEnglands.robot.systems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.BaseSubsystem;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.CommandLoop;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.servo.ServoImplExBase;

public class Transfer extends BaseSubsystem {
    @Configurable
    static class TransferPower {
        public static double power = 1;
        public static double reversePower = -1;
    }
    private DcMotorEx transfer;
    public Transfer(CommandLoop maps, HardwareMap hardwareMap) {
        super(maps);
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
