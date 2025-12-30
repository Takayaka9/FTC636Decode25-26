package org.firstinspires.ftc.teamcode.RIstates.management.handlers.fsm.states;

import org.firstinspires.ftc.teamcode.RIstates.management.handlers.fsm.State;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.fsm.FSM;
import org.firstinspires.ftc.teamcode.RIstates.management.SystemManager;

public class ShootState implements State {
    @Override
    public void initiate(SystemManager manager) {
        manager.shooter.stop();
    }

    @Override
    public void update(SystemManager manager, FSM fsm) {
        while (!manager.beltController.checkShotCounter()){
            manager.shooterController.shoot();
        }
        if (manager.beltController.checkShotCounter()) {
            fsm.runNew(FSM.StateName.Norm);
        }
    }

    @Override
    public void end(SystemManager manager) {
        manager.shooterController.off();
    }

}