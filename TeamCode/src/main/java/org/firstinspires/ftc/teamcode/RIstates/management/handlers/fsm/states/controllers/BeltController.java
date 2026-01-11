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
    private static int toleranceHIGH = 300; /// in TPS
    private static int toleranceLOW = -300;  /// In TPS

    public BeltController(Belt belt, Shooter shooter) {
        this.belt = belt;
        this.shooter = shooter;
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



}
