package org.firstinspires.ftc.teamcode.zRIstates;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.zRIstates.management.SystemManager;
@Disabled
@Configurable
@TeleOp(name = "States TeleOp 2", group = "TeleOp")
public class StatesTeleop2 extends LinearOpMode {

//    Subsystems + Follower
    SystemManager manager;



    public void runOpMode() throws InterruptedException {

        manager = new SystemManager(hardwareMap, telemetry, gamepad1, gamepad2, true, false);
        manager.init();

        waitForStart();
        if (isStopRequested()) return;

        manager.follower.update();
        manager.follower.startTeleopDrive();

        while(opModeIsActive()){
            manager.teleUpdate();
            manager.driveController.update();
        }
    }
}
