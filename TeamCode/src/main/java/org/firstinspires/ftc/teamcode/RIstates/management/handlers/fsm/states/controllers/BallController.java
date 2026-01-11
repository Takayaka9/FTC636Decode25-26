package org.firstinspires.ftc.teamcode.RIstates.management.handlers.fsm.states.controllers;

import org.firstinspires.ftc.teamcode.RIstates.management.handlers.fsm.states.controllers.subsystems.color.IntakeSensor;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.fsm.states.controllers.subsystems.color.TurretSensor;

public class BallController {

    IntakeSensor intake;
    TurretSensor turret;
    public BallController(IntakeSensor intake, TurretSensor turret) {
        this.intake = intake;
        this.turret = turret;
    }

    public int ballCount = 0;

    public boolean checkBotClear() {
        if (ballCount == 0) {
            return true;
        }
        return false;
    }

    public boolean checkBotFull() {
        if (ballCount == 0) {
            return true;
        }
        return false;
    }

    public int getBallCount() {
        return ballCount;
    }



    //call update method in loop
    boolean changedIntake = false;
    boolean changedTurret = false;

    public void intakeUpdateBallCount() {
        if (intake.CheckIsBallPresent() == IntakeSensor.detectedLocation.INTAKE) {
            if (!changedIntake) {
                ballCount++;
            }
            changedIntake = true;
        } else if (intake.CheckIsBallPresent() == IntakeSensor.detectedLocation.INTAKECLEAR) {
            changedIntake = false;
        }
    }

    public void shootUpdateBallCount() {
        if (turret.CheckIsBallPresent() == TurretSensor.detectedLocation.TURRET) {
            if (!changedTurret) {
                ballCount--;
            }
            changedTurret = true;
        } else if (turret.CheckIsBallPresent() == TurretSensor.detectedLocation.TURRETCLEAR) {
            changedTurret = false;
        }
    }
}
