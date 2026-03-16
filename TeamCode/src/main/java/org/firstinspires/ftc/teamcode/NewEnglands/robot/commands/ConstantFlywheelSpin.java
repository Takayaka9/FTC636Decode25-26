package org.firstinspires.ftc.teamcode.NewEnglands.robot.commands;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.NewEnglands.robot.systems.TakaShooter;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.alliance.CurrentAlliance;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.alliance.TDistHelper;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.BaseCommand;

public class ConstantFlywheelSpin extends BaseCommand {
    private final TakaShooter shooter;
    private final Follower follower;
    public ConstantFlywheelSpin(TakaShooter shooter, Follower follower) {
        super();
        this.shooter = shooter;
        this.follower = follower;
        addRequirement(shooter);
    }

    @Override
    public void init() {
        shooter.stop();
    }

    @Override
    public void loop() {
        shooter.runForDistance(TDistHelper.getTargetDistance(follower.getPose(), CurrentAlliance.alliance));
    }

    @Override
    public void stop() {
        shooter.stop();
    }
}
