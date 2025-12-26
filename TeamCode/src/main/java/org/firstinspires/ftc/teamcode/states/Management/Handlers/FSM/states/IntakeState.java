package org.firstinspires.ftc.teamcode.states.Management.Handlers.FSM.states;

import org.firstinspires.ftc.teamcode.states.Management.Handlers.FSM.State;
import org.firstinspires.ftc.teamcode.states.Management.Handlers.FSM.TeleOpFSM;
import org.firstinspires.ftc.teamcode.states.Management.SystemManager;

public class IntakeState implements State {

    @Override
    public void initiate(SystemManager manager) {
        manager.intake.run();
    }

    @Override
    public void update(SystemManager manager, TeleOpFSM fsm) {
    }

    @Override
    public void end(SystemManager manager) {
        manager.intake.stop();
    }

}