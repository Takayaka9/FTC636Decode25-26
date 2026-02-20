package org.firstinspires.ftc.teamcode.zRIstates.management.fsm.states;

import org.firstinspires.ftc.teamcode.zRIstates.management.fsm.State;
import org.firstinspires.ftc.teamcode.zRIstates.management.fsm.FSM;
import org.firstinspires.ftc.teamcode.zRIstates.management.SystemManager;

public class NormState implements State {

    @Override
    public void initiate(SystemManager manager) {
    }

    @Override
    public void update(SystemManager manager, FSM fsm) {
        //manager.shooterHandler.constantShoot();
    }

    @Override
    public void end(SystemManager manager) {
        //manager.shooterHandler.off();
    }

}