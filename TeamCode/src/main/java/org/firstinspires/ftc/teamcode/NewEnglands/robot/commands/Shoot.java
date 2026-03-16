package org.firstinspires.ftc.teamcode.NewEnglands.robot.commands;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.NewEnglands.robot.systems.TakaShooter;
import org.firstinspires.ftc.teamcode.NewEnglands.robot.systems.Transfer;
import org.firstinspires.ftc.teamcode.NewEnglands.robot.systems.servos.Stopper;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.alliance.TDistHelper;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.BaseCommand;

import java.util.concurrent.TimeUnit;

public class Shoot extends BaseCommand {
    private final Transfer transfer;
    private final TakaShooter shooter;
    private final Stopper stopper;
    private final Follower follower;
    private final TelemetryManager telemetryM;
    Timing.Timer stopperDelay;

    @Configurable
    private static class shootConfig {
        public static int delayTime = 1000;
    }

    public Shoot(Transfer transfer, TakaShooter shooter, Stopper stopper, Follower follower, TelemetryManager telemetryM) {
        super();
        //TODO: add shooter back into requirements if we need to
        this.transfer = transfer;
        this.shooter = shooter;
        this.stopper = stopper;
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
//        shooter.runForDistance(TDistHelper.TDistHelper(follower.getPose(), CurrentAlliance.alliance));
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
