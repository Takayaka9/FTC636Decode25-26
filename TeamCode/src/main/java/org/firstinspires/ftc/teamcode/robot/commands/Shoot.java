package org.firstinspires.ftc.teamcode.robot.commands;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.robot.systems.TakaShooter;
import org.firstinspires.ftc.teamcode.robot.systems.Transfer;
import org.firstinspires.ftc.teamcode.utils.alliance.CurrentAlliance;
import org.firstinspires.ftc.teamcode.utils.alliance.GetTargetDistance;
import org.firstinspires.ftc.teamcode.utils.commandUtils.BaseCommand;
import org.firstinspires.ftc.teamcode.utils.commandUtils.CommandLoop;

public class Shoot extends BaseCommand {
    Transfer transfer;
    TakaShooter shooter;
    GetTargetDistance getTargetDistance;
    Follower follower;
    TelemetryManager telemetryM;

    public Shoot(CommandLoop maps, Transfer transfer, TakaShooter shooter, Follower follower, TelemetryManager telemetryM) {
        super(maps);
        addRequirement(transfer, shooter);
        this.transfer = transfer;
        this.shooter = shooter;
        this.getTargetDistance = new GetTargetDistance();
        this.follower = follower;
        this.telemetryM = telemetryM;
    }

    @Override
    public void init() {
        transfer.stop();
        shooter.stop();
    }

    @Override
    public void loop() {
        shooter.runForDistance(getTargetDistance.getTargetDistance(follower.getPose(), CurrentAlliance.alliance));
        autoShoot();
    }

    @Override
    public void stop() {
        transfer.stop();
        shooter.stop();
    }

    public static double allowedError = 50;

    public boolean inRPMRange() {
        return Math.abs(shooter.getError()) < allowedError;
    }

    public void autoShoot() {
        if (inRPMRange()) {
            transfer.run();
        } else {
            transfer.stop();
        }
    }

}
