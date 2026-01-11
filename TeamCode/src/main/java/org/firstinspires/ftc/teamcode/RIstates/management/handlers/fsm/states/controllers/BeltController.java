package org.firstinspires.ftc.teamcode.RIstates.management.handlers.fsm.states.controllers;

import com.bylazar.configurables.annotations.Configurable;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.RIstates.management.handlers.fsm.states.controllers.subsystems.Belt;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.fsm.states.controllers.subsystems.Shooter;

import java.util.concurrent.TimeUnit;

@Configurable
public class BeltController {
    public final Belt belt;
    private final Shooter shooter;
    Timing.Timer runTimer;
    private static int runTime = 500;
    private static int toleranceHIGH = 300; /// in TPS
    private static int toleranceLOW = -300;  /// In TPS

    public BeltController(Belt belt, Shooter shooter) {
        this.belt = belt;
        this.shooter = shooter;
        runTimer = new Timing.Timer(runTime, TimeUnit.MILLISECONDS);
    }

    private int max = 0;
    private int min = 0;
    public boolean checkShooterReady() {
        max = shooter.averageVelocity() + toleranceHIGH;
        min = shooter.averageVelocity() + toleranceLOW;
        if (shooter.averageVelocity() < max && shooter.averageVelocity() > min) {
            return true;
        }
        return false;
    }


    public void run() {
        if (checkShooterReady()) {
            belt.run();
        } else {
            belt.stop();
        }
    }






//    public int shot = 0;
//    public boolean checkShotCounter() {
//        if (shot == 3) {
//            return true;
//        }
//        return false;
//    }
//    public void launch() {
//        while (!checkShooterReady()) {
//            runTimer.start();
//            belt.stop();
//        }
//        while (checkShooterReady()) {
//            if (!runTimer.done()) {
//                belt.run();
//            }
//            else if (runTimer.done()) {
//                shot++;
//                runTimer.start();
//            }
//        }
//    }
//
//    public boolean run() {
//        while (!checkShotCounter()) {
//            launch();
//        }
//        if (checkShotCounter()) {
//            shot = 0;
//            return true;
//        }
//        return false;
//    }


}
