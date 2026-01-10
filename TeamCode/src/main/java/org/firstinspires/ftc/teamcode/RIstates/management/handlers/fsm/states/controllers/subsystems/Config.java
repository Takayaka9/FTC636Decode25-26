package org.firstinspires.ftc.teamcode.RIstates.management.handlers.fsm.states.controllers.subsystems;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/*
This file is meant to store all of the information and components on the bot
so we can simply make an instance of this class in other files (such as auto, teleop)
and call each component.
 */
//TODO: ts is just copy and paste rn
@Configurable
public class Config {
    public DcMotorEx flyRight, flyLeft, intake; //motor declaration
//    public DcMotorEx belt; //idk why this is separate
    public DcMotorEx rightBack, rightFront, leftFront, leftBack;
//    public Servo onRamp, offRamp; //servos
    public RevColorSensorV3 colorSensor; //color sensor
//    public Limelight3A limelight;
    //TODO: try to put ALL values here...so we can use in both tele/auto!
    //TODO: we should put them in the subsystem classes themselves, i think we've been doing that though

    public Config(HardwareMap hardwareMap){
        rightBack = hardwareMap.get(DcMotorEx.class, "br");
        leftBack = hardwareMap.get(DcMotorEx.class, "bl");
        rightFront = hardwareMap.get(DcMotorEx.class, "fr");
        leftFront = hardwareMap.get(DcMotorEx.class, "fl");

        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        colorSensor.setGain(1);

//
    }

}
