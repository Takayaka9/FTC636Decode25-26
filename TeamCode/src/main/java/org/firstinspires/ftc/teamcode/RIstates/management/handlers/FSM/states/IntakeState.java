package org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.states;

import org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.State;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.FSM;
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