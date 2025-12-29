package org.firstinspires.ftc.teamcode.RIstates.management.handlers.fsm.states;

import org.firstinspires.ftc.teamcode.RIstates.management.handlers.fsm.State;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.fsm.FSM;
import org.firstinspires.ftc.teamcode.RIstates.management.SystemManager;

public class IntakeState implements State {

    @Override
    public void initiate(SystemManager manager) {
        manager.intake.run();
    }

    @Override
    public void update(SystemManager manager, FSM fsm) {
    }

    @Override
    public void end(SystemManager manager) {
        manager.intake.stop();
    }

}