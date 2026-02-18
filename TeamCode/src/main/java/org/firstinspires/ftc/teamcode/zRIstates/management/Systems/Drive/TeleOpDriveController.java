package org.firstinspires.ftc.teamcode.zRIstates.management.Systems.Drive;

import  com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.zRIstates.management.Systems.Controller;

public class TeleOpDriveController implements Controller {

    public void init() {};
    public void end() {};
    public errors updateError() {
        return errors.RUNNING;
    };

    private final Follower follower;
    private final Gamepad gamepad;

    public TeleOpDriveController(Follower follower, Gamepad gamepad) {
            this.follower = follower;
            this.gamepad = gamepad;
    }

    public void update () {

    }

}

