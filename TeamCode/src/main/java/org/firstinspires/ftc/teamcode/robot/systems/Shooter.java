package org.firstinspires.ftc.teamcode.robot.systems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.commandUtils.CommandLoop;
import org.firstinspires.ftc.teamcode.utils.commandUtils.BaseSubsystem;

public class Shooter extends BaseSubsystem {
    DcMotorEx turret;
    public Shooter(CommandLoop maps, HardwareMap hardwareMap) {
        super(maps);
        turret = hardwareMap.get(DcMotorEx.class, "turret");
    }


}
