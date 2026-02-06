package org.firstinspires.ftc.teamcode.RIstates.management.fsm.states;

import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Controller;
import org.firstinspires.ftc.teamcode.RIstates.management.fsm.State;
import org.firstinspires.ftc.teamcode.RIstates.management.fsm.FSM;
import org.firstinspires.ftc.teamcode.RIstates.management.SystemManager;

public class ShootState implements State {
    public boolean changed2X = false;
    @Override
    public void initiate(SystemManager manager) {
        manager.shooterHandler.shooterRunning = true;
    }

    @Override
    public void update(SystemManager manager, FSM fsm) {
        manager.shooterHandler.shoot();
        if (manager.gamepad2.x && !changed2X && manager.shooterHandler.inRange()) {
            manager.intakeController.run();
            changed2X = true;
        }
        else if (manager.gamepad2.left_bumper) {
            manager.intakeController.reverse();
            changed2X = true;
        }
        else if (!manager.gamepad2.x && !manager.gamepad2.right_bumper) {
            manager.intakeController.stop();
            changed2X = false;
        }
    }


    @Override
    public void end(SystemManager manager) {
        manager.shooterHandler.shooterRunning = false;
        manager.shooterHandler.off();
        manager.intakeController.stop();
    }

}