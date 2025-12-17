package org.firstinspires.ftc.teamcode.zpedroTesting;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "OdoTest", group = "Autonomous")
public class FirstOdoTest extends LinearOpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    // Hopefully declare motors?
    private DcMotor FRMotor;
    private DcMotor FLMotor;
    private DcMotor BRMotor;
    private DcMotor BLMotor;


    @Override
    public void runOpMode() {

        waitForStart();

        while (opModeIsActive()) {

            // Like this?
            FRMotor = hardwareMap.dcMotor.get("FRMotor");
            FLMotor = hardwareMap.dcMotor.get("FLMotor");
            BRMotor = hardwareMap.dcMotor.get("BRMotor");
            BLMotor = hardwareMap.dcMotor.get("BLMotor");
            // WIP
            if(gamepad1.right_trigger > 0.5) { // Forward
                FRMotor.setPower(1);
                FLMotor.setPower(1);
                BRMotor.setPower(0);
                BLMotor.setPower(0);
            }
            else if(gamepad1.left_trigger > 0.5) { // Brake
                FRMotor.setPower(0);
                FLMotor.setPower(0);
                BRMotor.setPower(0);
                BLMotor.setPower(0);
            }
            else if(gamepad1.right_trigger > 0.5 && gamepad1.right_bumper) { // Reverse
                FRMotor.setPower(0);
                FLMotor.setPower(0);
                BRMotor.setPower(1);
                BLMotor.setPower(1);
            }
        }
    }
}
