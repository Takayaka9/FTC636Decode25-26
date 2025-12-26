package org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.states;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.State;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.TeleOpFSM;
import org.firstinspires.ftc.teamcode.RIstates.management.SystemManager;

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
    public void update(SystemManager manager, TeleOpFSM fsm, Gamepad gamepad) {
    }

    @Override
    public void end(SystemManager manager) {
        manager.shooter.stop();
        manager.intake.stop();
    }

}

