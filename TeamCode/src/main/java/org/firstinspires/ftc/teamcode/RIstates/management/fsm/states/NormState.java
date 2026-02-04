package org.firstinspires.ftc.teamcode.RIstates.management.fsm.states;

import org.firstinspires.ftc.teamcode.RIstates.management.fsm.State;
import org.firstinspires.ftc.teamcode.RIstates.management.fsm.FSM;
import org.firstinspires.ftc.teamcode.RIstates.management.SystemManager;

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