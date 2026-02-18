package org.firstinspires.ftc.teamcode.zRIstates.management.fsm.states;

import org.firstinspires.ftc.teamcode.zRIstates.management.SystemManager;
import org.firstinspires.ftc.teamcode.zRIstates.management.fsm.State;
import org.firstinspires.ftc.teamcode.zRIstates.management.fsm.FSM;
import org.firstinspires.ftc.teamcode.zRIstates.management.handlers.TeleOpHandler;

public class AllianceSelectorState implements State {
    TeleOpHandler teleOpHandler;

    @Override
    public void initiate(SystemManager manager) {
        this.teleOpHandler = manager.teleOpHandler;
    }
    @Override
    public void update(SystemManager manager, FSM teleOpfsm) {
            if (manager.gamepad1.x) {
                manager.setAlliance(1);
                teleOpHandler.changedX = true;
            }
            if (manager.gamepad1.b) {
                manager.setAlliance(2);
                teleOpHandler.changedB = true;
            }
    }

    @Override
    public void end(SystemManager manager) {
    }
}
