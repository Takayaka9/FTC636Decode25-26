package org.firstinspires.ftc.teamcode.RIstates.management.Systems.ballController;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Controller;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.color.IntakeSensor;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.color.TurretSensor;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Utilities.timers.GenericTime;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Utilities.timers.SolversTiming;

@Configurable
public class BallController extends BallCounter implements Controller {
    GenericTime timer;
    private boolean shooting = false;
    private static int shootTimeMax = 4500;
    public BallController(IntakeSensor intake, TurretSensor turret) {
        super(intake, turret);
        timer = new SolversTiming();
        timer.create();
    }


    public void init() {
        if (shooting) {
            timer.resetThenStart();
        }
        timer.setLength(shootTimeMax);
    }

    public void setErrorHandler(boolean isShooting) {
        shooting = isShooting;
    }

    public void update(){
        if (shooting) {
            shootUpdateBallCount();
        } else {
            intakeUpdateBallCount();
        }
    }

    public void end() {}

    public errors updateError() {
        if (checkBotClear() && shooting) {
            return errors.ErrorCausesNormState;
        }
        if (checkBotFull() && !shooting) {
            return errors.ErrorCausesNormState;
        }
        if (shooting && timer.checkFinished()) {
            return errors.ErrorCausesNormState;
        }
        return errors.RUNNING;
    }



}
