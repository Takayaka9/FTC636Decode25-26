package org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.states;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.State;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.TeleOpFSM;
import org.firstinspires.ftc.teamcode.RIstates.management.SystemManager;

public class ShootState implements State {
    @Override
    public void initiate(SystemManager manager) {
        manager.shooter.stop();
    }

    @Override
    public void update(SystemManager manager, TeleOpFSM fsm, Gamepad gamepad) {
        //TODO: make alliance happen
        manager.shooterController.shoot(manager.alliance);
    }

    @Override
    public void end(SystemManager manager) {
        manager.shooter.stop();
    }

}