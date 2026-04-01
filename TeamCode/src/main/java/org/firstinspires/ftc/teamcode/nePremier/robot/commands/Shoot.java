package org.firstinspires.ftc.teamcode.nePremier.robot.commands;

import com.bylazar.configurables.annotations.Configurable;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.nePremier.robot.systems.TakaShooter;
import org.firstinspires.ftc.teamcode.nePremier.robot.systems.Transfer;
import org.firstinspires.ftc.teamcode.nePremier.robot.systems.servos.Stopper;
import org.firstinspires.ftc.teamcode.nePremier.utils.commandUtils.BaseCommand;

import java.util.concurrent.TimeUnit;

public class Shoot extends BaseCommand {
    private final Transfer transfer;
    private final TakaShooter shooter;
    private final Stopper stopper;
    private final Timing.Timer stopperDelay;

    @Configurable
    public static class shootConfig {
        public static int delayTime = 300;
        public static double allowedError = 50;
    }

    public Shoot(Transfer transfer, TakaShooter shooter, Stopper stopper) {
        super();
        //TODO: add shooter back into requirements if we need to
        this.transfer = transfer;
        this.shooter = shooter;
        this.stopper = stopper;
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

    public boolean inRPMRange() {
        return Math.abs(shooter.getError()) < shootConfig.allowedError;
    }

    public void autoShoot() {
        if (inRPMRange() && stopperDelay.done()) {
            transfer.run();
        } else {
            transfer.stop();
        }
    }

}
