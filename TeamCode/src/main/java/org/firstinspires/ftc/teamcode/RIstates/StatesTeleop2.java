package org.firstinspires.ftc.teamcode.RIstates;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RIstates.management.SystemManager;

@Configurable
@TeleOp(name = "States TeleOp", group = "TeleOp")
public class StatesTeleop2 extends LinearOpMode {

//    Subsystems + Follower
    SystemManager manager;
    @Override
    public void runOpMode() throws InterruptedException {

//        Subs init
        manager = new SystemManager(hardwareMap, gamepad1, gamepad2, true);

        waitForStart();
        if (isStopRequested()) return;

        manager.follower.update();
        manager.follower.startTeleopDrive();

        while(opModeIsActive()){
            manager.teleUpdate();
            manager.driveController.teleopNorm();
        }
    }
}
