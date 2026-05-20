package org.firstinspires.ftc.teamcode.nePremier.robot.commands;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.nePremier.robot.systems.BotPose;
import org.firstinspires.ftc.teamcode.nePremier.robot.systems.TakaShooter;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.LocalizationHelper;
import org.firstinspires.ftc.teamcode.nePremier.utils.commandUtils.BaseCommand;

public class ConstantFlywheelSpin extends BaseCommand {
    private final TakaShooter shooter;
    private final Follower follower;
    private final BotPose botPose;
    public ConstantFlywheelSpin(TakaShooter shooter, BotPose botpose, Follower follower) {
        super();
        this.shooter = shooter;
        this.botPose = botpose;
        this.follower = follower;
    }

    @Override
    public void init() {
        shooter.stop();
    }

    @Override
    public void loop() {
//        shooter.runForDistance(LocalizationHelper.getTargetDistance(botPose.getBotPose()));
        shooter.test(1200);
    }

    @Override
    public void stop() {
        shooter.stop();
    }
}
