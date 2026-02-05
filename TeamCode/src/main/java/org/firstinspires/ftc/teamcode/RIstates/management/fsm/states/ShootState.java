package org.firstinspires.ftc.teamcode.RIstates.management.fsm.states;

import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Controller;
import org.firstinspires.ftc.teamcode.RIstates.management.fsm.State;
import org.firstinspires.ftc.teamcode.RIstates.management.fsm.FSM;
import org.firstinspires.ftc.teamcode.RIstates.management.SystemManager;

import java.util.concurrent.TimeUnit;

public class ShootState implements State {
//    Timing.Timer timer;
    public boolean changed2X = false;
    @Override
    public void initiate(SystemManager manager) {
        //manager.shooterHandler.off();
//        manager.ballController.setErrorHandler(true);
//        manager.ballController.init();
        manager.shooterHandler.shooterRunning = true;
        //manager.gateServo.togglePosition(false);
        // TODO: manager.gateServo.setPosition(notblocking);
//        timer = new Timing.Timer(500, TimeUnit.MILLISECONDS);
//        manager.intake.reverse();
    }

    @Override
    public void update(SystemManager manager, FSM fsm) {
//        manager.ballController.update();
//        if (manager.ballController.updateError() == Controller.errors.RUNNING & manager.ballController.updateError() == Controller.errors.ContinueWithError) {
//            manager.shooterHandler.shoot();
//        } else if (manager.ballController.updateError() == Controller.errors.ErrorCausesNormState) {
//            manager.shooterHandler.shooterRunning = false;
//            manager.FSM.runNew(FSM.StateName.Norm);
//        }

//        if (timer.done()) {
//            manager.intake.stop();
//            manager.shooterHandler.shoot();
//        }
        manager.shooterHandler.shoot();
//        manager.shooterHandler.whileConstantShootingBelt(true);
        if (manager.gamepad2.x && !changed2X && manager.shooterHandler.inRange()) {
            manager.intakeController.shootRun();
            changed2X = true;
        } else if (!manager.gamepad2.x && changed2X) {
            manager.intakeController.stop();
            changed2X = false;
        }
    }


    @Override
    public void end(SystemManager manager) {
        manager.shooterHandler.shooterRunning = false;
        manager.shooterHandler.off();
        manager.intakeController.stop();
        //manager.shooterHandler.whileConstantShootingBelt(true);
        manager.gateServo.togglePosition(true);
        //TODO: manager.gateServo.setPosition(blocking);
    }

}