package org.firstinspires.ftc.teamcode.RIstates.management.Systems.Drive;

import  com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;

public class TeleOpDriveController {

    private final Follower follower;
    private final Gamepad gamepad;

    public TeleOpDriveController(Follower follower, Gamepad gamepad) {
            this.follower = follower;
            this.gamepad = gamepad;
    }

    public void teleopNorm () {
        if (gamepad.left_trigger < 0.3) {
            follower.setTeleOpDrive(
                    -gamepad.left_stick_y*1,
                    -gamepad.left_stick_x*1,
                    -gamepad.right_stick_x*0.45,
                    true
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

