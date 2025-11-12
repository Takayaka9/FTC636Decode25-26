package org.firstinspires.ftc.teamcode.scrims;


import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.PIDFControl_ForVelocity;

public class RobotScrims {
    public DcMotorEx flyRight, flyLeft, intake;
    public DcMotorEx belt;
    public Servo onRamp, offRamp;
    public static int onRampPassive = 0;
    public static int onRampPush = 0;
    public static int offRampPassive = 0;
    public static int offRampPush = 0;
    public ColorSensor colorSensor;
    public DistanceSensor distanceSensor;
    public RobotScrims(HardwareMap hardwareMap){
        flyRight = hardwareMap.get(DcMotorEx.class, "flyRight");
        flyLeft = hardwareMap.get(DcMotorEx.class, "flyLeft");
        intake = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        belt = hardwareMap.get(DcMotorEx.class, "beltMotor");

        onRamp = hardwareMap.get(Servo.class, "onRamp");
        offRamp = hardwareMap.get(Servo.class, "offRamp");

        flyRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        belt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flyRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flyLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        belt.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        colorSensor = hardwareMap.get(ColorSensor.class, "CDSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "CDSensor");
    }

    public void initialTele(){
        onRamp.setPosition(onRampPassive);
        offRamp.setPosition(offRampPassive);
    }

    public void shoot(double velocity){
        PIDFControl_ForVelocity control = new PIDFControl_ForVelocity(0.0, 0.0, 0.0, 0.0);

        double powerLeft = control.update(velocity, flyLeft.getVelocity());
        double powerRight = control.update(velocity, flyRight.getVelocity());

        flyLeft.setPower(powerLeft);
        flyRight.setPower(powerRight);
    }

    public void pushOff(){
        onRamp.setPosition(onRampPush);
        onRamp.setPosition(onRampPassive);
    }

    public void pushOn(){
        offRamp.setPosition(offRampPush);
        offRamp.setPosition(offRampPassive);
    }

    public boolean isBallThere(){
        return distanceSensor.getDistance(DistanceUnit.CM) < 1;
    }

    
}
