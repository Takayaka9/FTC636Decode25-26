package org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.states;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RIstates.management.SystemManager;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.State;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.TeleOpFSM;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.TeleOpHandler;

public class AllianceSelectorState implements State {
    @Override
    public void initiate(SystemManager manager) {
    }

    TeleOpHandler TeleOpHandler;
    @Override
    public void update(SystemManager manager, TeleOpFSM TeleOpfsm) {
        if (TeleOpfsm != null && TeleOpHandler != null){
            if (manager.gamepad1.x && !TeleOpHandler.changedX && !TeleOpHandler.changedB) {
                manager.setAlliance(1);
                TeleOpHandler.changedX = true;
            }
            if (manager.gamepad1.b && !TeleOpHandler.changedB && !TeleOpHandler.changedX) {
                manager.setAlliance(2);
                TeleOpHandler.changedB = true;
            }
        }
    }

    @Override
    public void end(SystemManager manager) {
    }
}
