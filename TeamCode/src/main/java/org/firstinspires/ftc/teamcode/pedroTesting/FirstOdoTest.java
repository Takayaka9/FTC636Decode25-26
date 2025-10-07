package org.firstinspires.ftc.teamcode.pedroTesting;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FirstOdoTest extends LinearOpMode {

    // Declare motors
    DcMotor FLMotor;
    DcMotor FRMotor;
    DcMotor BLMotor;
    DcMotor BRMotor;

    @Override
    public void runOpMode() {

        FLMotor = hardwareMap.get(DcMotor.class, "FLMotor");
        FRMotor = hardwareMap.get(DcMotor.class, "FRMotor");
        BLMotor = hardwareMap.get(DcMotor.class, "BLMotor");
        BRMotor = hardwareMap.get(DcMotor.class, "BRMotor");

        // Directions
        FRMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BLMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Encoders
        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Brakes
        FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {

            // Write the rest of code...



        }
    }
}
