package org.firstinspires.ftc.teamcode.RIstates.management.fsm.states;

import org.firstinspires.ftc.teamcode.RIstates.management.fsm.State;
import org.firstinspires.ftc.teamcode.RIstates.management.fsm.FSM;
import org.firstinspires.ftc.teamcode.RIstates.management.SystemManager;

public class BackoutState implements State {

    @Override
    public void initiate(SystemManager manager) {
        manager.intake.reverse();
        manager.shooter.reverse();
    }

    @Override
    public void update(SystemManager manager, FSM fsm) {
    }

    @Override
    public void end(SystemManager manager) {
        manager.shooter.stop();
        manager.intake.stop();
    }

}

