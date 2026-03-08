package org.firstinspires.ftc.teamcode.NewEnglands.robot.commands;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.NewEnglands.robot.systems.TakaShooter;
import org.firstinspires.ftc.teamcode.NewEnglands.robot.systems.Transfer;
import org.firstinspires.ftc.teamcode.NewEnglands.robot.systems.servos.Stopper;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.alliance.GetTargetDistance;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.BaseCommand;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.CommandLoop;

import java.util.concurrent.TimeUnit;

public class Shoot extends BaseCommand {
    private final Transfer transfer;
    private final TakaShooter shooter;
    private final Stopper stopper;
    private final GetTargetDistance getTargetDistance;
    private final Follower follower;
    private final TelemetryManager telemetryM;
    Timing.Timer stopperDelay;

    @Configurable
    private static class shootConfig {
        public static int delayTime = 1000;
    }

    public Shoot(CommandLoop maps, Transfer transfer, TakaShooter shooter, Stopper stopper, Follower follower, TelemetryManager telemetryM) {
        super();
        //TODO: add shooter back into requirements if we need to
        addRequirement(transfer, stopper);
        this.transfer = transfer;
        this.shooter = shooter;
        this.stopper = stopper;
        this.getTargetDistance = new GetTargetDistance();
        this.follower = follower;
        this.telemetryM = telemetryM;
        stopperDelay = new Timing.Timer(shootConfig.delayTime, TimeUnit.MILLISECONDS);
    }

    @Override
    public void init() {
        stopperDelay.start();
        stopper.open();
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
        stopper.close();
        transfer.stop();
//        shooter.stop();
    }

    public static double allowedError = 50;

    public boolean inRPMRange() {
        return Math.abs(shooter.getError()) < allowedError;
    }

    public void autoShoot() {
        if (inRPMRange() && stopperDelay.done()) {
            transfer.run();
        } else {
            transfer.stop();
        }
    }

}
