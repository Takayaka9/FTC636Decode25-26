package org.firstinspires.ftc.teamcode.RIstates.management.fsm.states;

import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Controller;
import org.firstinspires.ftc.teamcode.RIstates.management.fsm.State;
import org.firstinspires.ftc.teamcode.RIstates.management.fsm.FSM;
import org.firstinspires.ftc.teamcode.RIstates.management.SystemManager;

public class ShootState implements State {
    @Override
    public void initiate(SystemManager manager) {
        //manager.shooterHandler.off();
//        manager.ballController.setErrorHandler(true);
//        manager.ballController.init();
        manager.shooterHandler.shooterRunning = true;
        manager.gateServo.togglePosition(false);
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
        manager.shooterHandler.shoot();
//        manager.shooterHandler.whileConstantShootingBelt(true);
    }

    @Override
    public void end(SystemManager manager) {
        manager.shooterHandler.shooterRunning = false;
        manager.shooterHandler.off();
        //manager.shooterHandler.whileConstantShootingBelt(true);
        manager.gateServo.togglePosition(true);
    }

}