package org.firstinspires.ftc.teamcode.nePremier.opModes.tests;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.nePremier.robot.commands.MakeMoves;
import org.firstinspires.ftc.teamcode.nePremier.utils.inputSystem.Control;
import org.firstinspires.ftc.teamcode.nePremier.utils.inputSystem.ControlType;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class EmadWantsToHaveFun extends OpMode {
    Follower follower;
    MakeMoves makeMoves;
    Control control;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        makeMoves = new MakeMoves(follower, gamepad1);
        control = new Control(ControlType.Auto, makeMoves);
        control.run();
    }

    @Override
    public void loop() {
        control.update();
        follower.update();
    }
}

