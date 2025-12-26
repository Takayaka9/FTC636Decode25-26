package org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.states.controllers;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TeleOpDriveController {

    private HardwareMap hardwareMap;
    private Follower follower;
    private Gamepad gamepad;

    public TeleOpDriveController(HardwareMap hardwareMap, Follower follower, Gamepad gamepad) {
            this.hardwareMap = hardwareMap;
            this.follower = follower;
            this.gamepad = gamepad;
    }

    public void teleopNorm () {
        if (this.gamepad.left_trigger < 0.3) {
            this.follower.setTeleOpDrive(
                    -gamepad.left_stick_y*1,
                    -gamepad.left_stick_x*1,
                    -gamepad.right_stick_x*0.45, false
            );
        }
        if (this.gamepad.left_trigger > 0.3) {
            this.follower.setTeleOpDrive(
                    -this.gamepad.left_stick_y*0.35,
                    -this.gamepad.left_stick_x*0.35,
                    -this.gamepad.right_stick_x*0.25,
                    false
            );
        }
    }

}

