package org.firstinspires.ftc.teamcode.zRIstates.management.fsm.states;

import org.firstinspires.ftc.teamcode.zRIstates.management.fsm.State;
import org.firstinspires.ftc.teamcode.zRIstates.management.fsm.FSM;
import org.firstinspires.ftc.teamcode.zRIstates.management.SystemManager;

public class AutoShootState implements State {
    @Override
    public void initiate(SystemManager manager) {
        manager.shooterHandler.shooterRunning = true;
    }

    @Override
    public void update(SystemManager manager, FSM tFSM) {
        manager.shooterHandler.shoot();
        manager.shooterHandler.autoShoot();
    }

    @Override
    public void end(SystemManager manager) {
        manager.shooterHandler.shooterRunning = false;
        manager.shooterHandler.off();
        manager.intakeController.stop();
    }
}
