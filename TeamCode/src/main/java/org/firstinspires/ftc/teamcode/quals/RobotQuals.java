package org.firstinspires.ftc.teamcode.quals;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

/*
This file is meant to store all of the information and components on the bot
so we can simply make an instance of this class in other files (such as auto, teleop)
and call each component.
 */
@Configurable
public class RobotQuals {
    public DcMotorEx flyRight, flyLeft, intake; //motor declaration
    public DcMotorEx belt; //idk why this is separate
    public Servo onRamp, offRamp; //servos
    public RevColorSensorV3 colorSensor; //color sensor
    //public DistanceSensor distanceSensor; //distance sensor (same as color)
    public Limelight3A limelight;
    public static int onRampPassive = 0; //TODO: test values
    public static int onRampPush = 0;
    public static int offRampPassive = 0;
    public static int offRampPush = 0;
    public static double intakePower = 0.7;
    public static double beltPower = 0.5;
    public static double beltBackPower = -0.4;
    double velocity;
    public double TICKS_PER_REV = 24;
    public double shooterOutput;
    //Note: values are examples from ftclib docs
    public static double kP = 0.37;
    public static double kI = 0.37;
    public static double kD = 0.37;
    public static double kF = 0.37;
    public static double kS = 0;
    public static double kV = 0;
    public static double kA = 0;

    PIDFController pidf = new PIDFController(kP, kI, kD, kF);
    //old: public static PIDFControl_ForVelocity shootControl = new PIDFControl_ForVelocity(1.19, 2.0, 1.1, 0.0);
    // HOPEFULLY TUNED RIGHT??? ALL VALUES WERE 0 BEFORE IF ISSUES ARISE

    public RobotQuals(HardwareMap hardwareMap){
        flyRight = hardwareMap.get(DcMotorEx.class, "flyRight");
        flyLeft = hardwareMap.get(DcMotorEx.class, "flyLeft");
        intake = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        belt = hardwareMap.get(DcMotorEx.class, "beltMotor");

        onRamp = hardwareMap.get(Servo.class, "onRamp");
        //offRamp = hardwareMap.get(Servo.class, "offRamp");

        flyRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        belt.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        intake.setDirection(DcMotorEx.Direction.REVERSE);
        flyLeft.setDirection(DcMotorEx.Direction.REVERSE);

        flyRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flyLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        belt.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        colorSensor.setGain(1);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        //telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        //IMU imu = hardwareMap.get(IMU.class, "imu");
    }

    //initial positions of everything at the start of teleop, add as needed
    /*
    public void initialTele(){
        onRamp.setPosition(onRampPassive);
        offRamp.setPosition(offRampPassive);
    }

     */


    /*
    public enum DetectedColor{
        GREEN,
        PURPLE,
        UNKNOWN
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


    //Color sensor by Taka, -Emad
    //moved to the actual teleop since I think that's better
    /*
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

     */

//New ftc lib pid:
public double shooterPIDF(double desiredRPM) {
    //PIDF
    pidf.setSetPoint(desiredRPM);
    double measuredVelocity = flyRight.getVelocity();
    double outputPIDF = pidf.calculate(measuredVelocity, desiredRPM);
    //Feedforward:
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);
    double outputFF = feedforward.calculate(desiredRPM);

    //add and set power
    double output = outputPIDF + outputFF;
    flyRight.setPower(output);
    flyLeft.setPower(output);

    return output;
}


    //method to push a ball off of the ramp
    /*
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

    //Code graveyard:
    /*
    //spins shooter using PIDF control based on the target velocity that is passed through as a parameter
    public double RPMtoVelocity(int targetRPM) {
        return (targetRPM * TICKS_PER_REV)/60;
    }

    public void shoot(int RPM) {
        velocity = RPMtoVelocity(RPM);
        flyRight.setVelocity(velocity);
        flyLeft.setVelocity(velocity);
    }
*/

}
