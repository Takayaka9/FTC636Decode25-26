package org.firstinspires.ftc.teamcode.scrims;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.PIDFControl_ForVelocity;
/*
This file is meant to store all of the information and components on the bot
so we can simply make an instance of this class in other files (such as auto, teleop)
and call each component.
 */
@Configurable
public class RobotScrims {
    public DcMotorEx flyRight, flyLeft, intake; //motor declaration
    public DcMotorEx belt; //idk why this is separate
    public Servo onRamp, offRamp; //servos
    public NormalizedColorSensor colorSensor; //color sensor
    //public DistanceSensor distanceSensor; //distance sensor (same as color)
    Limelight3A limelight3A; //limelight
    public static int onRampPassive = 0; //TODO: test values
    public static int onRampPush = 0;
    public static int offRampPassive = 0;
    public static int offRampPush = 0;
    public static double intakePower = 0.7;
    public static double beltPower = 0.5;
    public static double beltBackPower = -0.4;
    double velocity;
    public double TICKS_PER_REV = 24;
    public static PIDFControl_ForVelocity shootControl = new PIDFControl_ForVelocity(0.0, 0.0, 0.0, 0.0); //TODO: TUNE PIDF VALUES
    public RobotScrims(HardwareMap hardwareMap){
        flyRight = hardwareMap.get(DcMotorEx.class, "flyRight");
        flyLeft = hardwareMap.get(DcMotorEx.class, "flyLeft");
        intake = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        belt = hardwareMap.get(DcMotorEx.class, "beltMotor");

        //onRamp = hardwareMap.get(Servo.class, "onRamp");
        //offRamp = hardwareMap.get(Servo.class, "offRamp");

        flyRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        belt.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorEx.Direction.REVERSE);

        flyRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flyLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        belt.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        //belt.setTargetPosition(belt.getCurrentPosition());


       // colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        //colorSensor.setGain(1);

        //limelight3A = hardwareMap.get(Limelight3A.class, "limelight");

    }

    //initial positions of everything at the start of teleop, as add needed
    public void initialTele(){
        //onRamp.setPosition(onRampPassive);
        //koffRamp.setPosition(offRampPassive);
    }

    public enum DetectedColor{
        GREEN,
        PURPLE,
        UNKNOWN
    }

    public DetectedColor getDetectedColor(TelemetryManager telemetryManager){
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        float normRed, normGreen, normBlue;

        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;

        //TODO: calibrate the thresholds and add if statements

        telemetryManager.addData("Red", normRed);
        telemetryManager.addData("Green", normGreen);
        telemetryManager.addData("Blue", normBlue);

        return DetectedColor.UNKNOWN;
    }

    //spins shooter using PIDF control based on the target velocity that is passed through as a parameter
    public double RPMtoVelocity (int targetRPM) {
        return (targetRPM * TICKS_PER_REV)/60;
    }

    public void shoot(int RPM) {
        velocity = RPMtoVelocity(RPM);
        flyRight.setVelocity(velocity);
        flyLeft.setVelocity(velocity);
    }

    /*
    //method to push a ball off of the ramp
    public void pushOff(){
        onRamp.setPosition(onRampPush);
        onRamp.setPosition(onRampPassive);
    }

    //method to push a ball back on the ramp
    public void pushOn(){
        offRamp.setPosition(offRampPush);
        offRamp.setPosition(offRampPassive);
    }
    
     */

    //run intake
    public void intakeRun(){
        intake.setPower(intakePower);
    }
    public void beltRun() {
        belt.setPower(beltPower);
    }
    public void beltBackRun() {
        belt.setPower(beltBackPower);
    }
}
