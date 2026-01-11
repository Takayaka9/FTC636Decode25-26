package org.firstinspires.ftc.teamcode.RIstates.management.fsm.states;

import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Controller;
import org.firstinspires.ftc.teamcode.RIstates.management.fsm.State;
import org.firstinspires.ftc.teamcode.RIstates.management.fsm.FSM;
import org.firstinspires.ftc.teamcode.RIstates.management.SystemManager;

public class ShootState implements State {
    @Override
    public void initiate(SystemManager manager) {
        manager.shooterHandler.off();
        manager.ballController.setErrorHandler(true);
        manager.ballController.init();
        manager.shooterHandler.shooterRunning = true;
    }

    @Override
    public void update(SystemManager manager, FSM fsm) {
        manager.ballController.update();
        if (manager.ballController.updateError() == Controller.errors.RUNNING & manager.ballController.updateError() == Controller.errors.ContinueWithError) {
            manager.shooterHandler.shoot();
        } else if (manager.ballController.updateError() == Controller.errors.ErrorCausesNormState) {
            manager.shooterHandler.shooterRunning = false;
            manager.FSM.runNew(FSM.StateName.Norm);
        }
    }

    @Override
    public void end(SystemManager manager) {
        manager.shooterHandler.shooterRunning = true;
        manager.shooterHandler.off();
    }

}