package org.firstinspires.ftc.teamcode.nePremier.robot.commands;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.nePremier.robot.systems.BotPose;
import org.firstinspires.ftc.teamcode.nePremier.robot.systems.TakaShooter;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.LocalizationHelper;
import org.firstinspires.ftc.teamcode.nePremier.utils.commandUtils.BaseCommand;

public class ConstantFlywheelSpin extends BaseCommand {
    private final TakaShooter shooter;
    private final Follower follower;
    private final BotPose botPose;
    private final Gamepad gamepad;
    int setTPS = 1000;
    public ConstantFlywheelSpin(TakaShooter shooter, BotPose botpose, Follower follower, Gamepad gamepad) {
        super();
        this.shooter = shooter;
        this.botPose = botpose;
        this.follower = follower;
        this.gamepad = gamepad;
    }

    @Override
    public void init() {
        shooter.stop();
    }

    @Override
    public void loop() {
//        shooter.runForDistance(LocalizationHelper.getTargetDistance(botPose.getBotPose()));
        if(gamepad.x) setTPS += 10;
        if(gamepad.b) setTPS -= 10;
        setTPS = Math.max(setTPS, 900);
        setTPS = Math.min(setTPS, 1450);
        shooter.test(setTPS);
    }

    @Override
    public void stop() {
        shooter.stop();
    }
}
