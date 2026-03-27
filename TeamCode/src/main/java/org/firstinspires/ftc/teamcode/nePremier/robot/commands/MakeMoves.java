package org.firstinspires.ftc.teamcode.nePremier.robot.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.nePremier.utils.commandUtils.BaseCommand;

public class MakeMoves extends BaseCommand {
    private final Follower follower;
    private final Gamepad gamepad;

    public MakeMoves(Follower follower, Gamepad gamepad1) {
        super();
        this.follower = follower;
        this.gamepad = gamepad1;
        this.follower.setStartingPose(new Pose(72, 72, 0));
        this.follower.update();
    }

    @Override
    public void init() {
        follower.startTeleOpDrive();
    }

    @Override
    public void loop() {
        if (gamepad != null) {
            if (gamepad.left_trigger < 0.3) {
                follower.setTeleOpDrive(
                        -gamepad.left_stick_y*1,
                        -gamepad.left_stick_x*1,
                        -gamepad.right_stick_x*0.75,
                        true
                );
            }
            if (gamepad.left_trigger > 0.3) {
                follower.setTeleOpDrive(
                        -gamepad.left_stick_y*0.35,
                        -gamepad.left_stick_x*0.35,
                        -gamepad.right_stick_x*0.25,
                        true
                );
            }
        }
    }

}
