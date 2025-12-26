package org.firstinspires.ftc.teamcode.states.Management.Handlers.FSM.states.Controllers.subsystems;


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
public class HardwareDependencies {
    public DcMotorEx flyRight, flyLeft, intake; //motor declaration
    public DcMotorEx belt; //idk why this is separate
    public DcMotorEx rightBack, rightFront, leftFront, leftBack;
    public Servo onRamp, offRamp; //servos
    public RevColorSensorV3 colorSensor; //color sensor
    public Limelight3A limelight;
    //TODO: try to put ALL values here...so we can use in both tele/auto!

    public HardwareDependencies(HardwareMap hardwareMap){
        flyRight = hardwareMap.get(DcMotorEx.class, "flyRight");
        flyLeft = hardwareMap.get(DcMotorEx.class, "flyLeft");
        intake = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        belt = hardwareMap.get(DcMotorEx.class, "beltMotor");

        //comment ts out if it tweaks
        //rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        //leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        //rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        //leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");

        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        onRamp = hardwareMap.get(Servo.class, "onRamp");
        offRamp = hardwareMap.get(Servo.class, "offRamp");

        flyRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        belt.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        intake.setDirection(DcMotorEx.Direction.REVERSE);
        flyRight.setDirection(DcMotorEx.Direction.REVERSE);

        flyRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flyLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        belt.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        colorSensor.setGain(1);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
    }

}
