package org.firstinspires.ftc.teamcode.RIstates.management.handlers.fsm.states.controllers;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TeleOpDriveController {

    private Follower follower;
    private Gamepad gamepad;

    public TeleOpDriveController(Follower follower, Gamepad gamepad) {
            this.follower = follower;
            this.gamepad = gamepad;
    }

    public void teleopNorm () {
        if (gamepad.left_trigger < 0.3) {
            follower.setTeleOpDrive(
                    -gamepad.left_stick_y*1,
                    -gamepad.left_stick_x*1,
                    -gamepad.right_stick_x*0.45, false
            );
        }
        if (gamepad.left_trigger > 0.3) {
            follower.setTeleOpDrive(
                    -gamepad.left_stick_y*0.35,
                    -gamepad.left_stick_x*0.35,
                    -gamepad.right_stick_x*0.25,
                    false
            );
        }
    }

}

