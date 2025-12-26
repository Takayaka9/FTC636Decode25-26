package org.firstinspires.ftc.teamcode.states.Management.Handlers.FSM.states;

import org.firstinspires.ftc.teamcode.states.Management.Handlers.FSM.State;
import org.firstinspires.ftc.teamcode.states.Management.Handlers.FSM.TeleOpFSM;
import org.firstinspires.ftc.teamcode.states.Management.SystemManager;

public class BackoutState implements State {
//    public void BackoutState(HardwareMap hardwareMap, SystemManager manager){
//        this.hardwareMap = new HardwareMap();
//        this.manager =
//    }

    @Override
    public void initiate(SystemManager manager) {
        manager.intake.reverse();
        manager.shooter.reverse();
    }

    @Override
    public void update(SystemManager manager, TeleOpFSM fsm) {
    }

    @Override
    public void end(SystemManager manager) {
        manager.shooter.stop();
        manager.intake.stop();
    }

}

