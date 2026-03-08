package org.firstinspires.ftc.teamcode.NewEnglands.robot.commands;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.NewEnglands.robot.systems.TakaShooter;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.alliance.CurrentAlliance;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.alliance.GetTargetDistance;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.BaseCommand;

public class ConstantShoot extends BaseCommand {
    private final TakaShooter shooter;
    private final GetTargetDistance getTargetDistance;
    private final Follower follower;
    public ConstantShoot(TakaShooter shooter, Follower follower) {
        super();
        getTargetDistance = new GetTargetDistance();
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
        shooter.runForDistance(getTargetDistance.getTargetDistance(follower.getPose(), CurrentAlliance.alliance));
    }

    @Override
    public void stop() {
        shooter.stop();
    }
}
