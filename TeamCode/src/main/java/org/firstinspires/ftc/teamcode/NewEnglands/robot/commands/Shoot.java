package org.firstinspires.ftc.teamcode.NewEnglands.robot.commands;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.NewEnglands.robot.systems.TakaShooter;
import org.firstinspires.ftc.teamcode.NewEnglands.robot.systems.Transfer;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.alliance.CurrentAlliance;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.alliance.GetTargetDistance;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.BaseCommand;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.CommandLoop;

public class Shoot extends BaseCommand {
    Transfer transfer;
    TakaShooter shooter;
    GetTargetDistance getTargetDistance;
    Follower follower;
    TelemetryManager telemetryM;

    public Shoot(CommandLoop maps, Transfer transfer, TakaShooter shooter, Follower follower, TelemetryManager telemetryM) {
        super(maps);
        //TODO: add shooter back into requirements if we need to
        addRequirement(transfer);
        this.transfer = transfer;
        this.shooter = shooter;
        this.getTargetDistance = new GetTargetDistance();
        this.follower = follower;
        this.telemetryM = telemetryM;
    }

    @Override
    public void init() {
        transfer.stop();
//        shooter.stop();
    }

    @Override
    public void loop() {
//        shooter.runForDistance(getTargetDistance.getTargetDistance(follower.getPose(), CurrentAlliance.alliance));
        autoShoot();
    }

    @Override
    public void stop() {
        transfer.stop();
//        shooter.stop();
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
