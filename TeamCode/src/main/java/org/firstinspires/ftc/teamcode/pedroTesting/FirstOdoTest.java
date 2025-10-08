package org.firstinspires.ftc.teamcode.pedroTesting;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class FirstOdoTest extends LinearOpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    // Declare motors


    @Override
    public void runOpMode() {



        waitForStart();

        while (opModeIsActive()) {

            // Write the rest of code...



        }
    }
}
