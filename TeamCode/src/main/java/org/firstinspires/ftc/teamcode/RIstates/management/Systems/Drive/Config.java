package org.firstinspires.ftc.teamcode.RIstates.management.Systems.Drive;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
This file is meant to store all of the information and components on the bot
so we can simply make an instance of this class in other files (such as auto, teleop)
and call each component.
 */
//TODO: ts is just copy and paste rn
@Configurable
public class Config {
    public DcMotorEx rightBack, rightFront, leftFront, leftBack;
    public Config(HardwareMap hardwareMap){
        rightBack = hardwareMap.get(DcMotorEx.class, "br");
        leftBack = hardwareMap.get(DcMotorEx.class, "bl");
        rightFront = hardwareMap.get(DcMotorEx.class, "fr");
        leftFront = hardwareMap.get(DcMotorEx.class, "fl");

        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//
    }

}
