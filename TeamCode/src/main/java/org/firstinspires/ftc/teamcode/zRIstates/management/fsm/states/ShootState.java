package org.firstinspires.ftc.teamcode.zRIstates.management.fsm.states;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.zRIstates.management.fsm.State;
import org.firstinspires.ftc.teamcode.zRIstates.management.fsm.FSM;
import org.firstinspires.ftc.teamcode.zRIstates.management.SystemManager;

public class ShootState implements State {
    public boolean changed2X = false;
    ElapsedTime reverseTime = new ElapsedTime();
    @Override
    public void initiate(SystemManager manager) {
        manager.shooterHandler.shooterRunning = true;
    }

    @Override
    public void update(SystemManager manager, FSM fsm) {
        manager.shooterHandler.shoot();
        if (manager.gamepad2.x && !changed2X && manager.shooterHandler.inRange()) {
            manager.intakeController.run();
            reverseTime.reset();
            changed2X = true;
        }
        else if (manager.gamepad2.left_bumper) {
            manager.intakeController.reverseABit(reverseTime);
            changed2X = true;
        }
        else if (!manager.gamepad2.x && !manager.gamepad2.right_bumper) {
            manager.intakeController.stop();
            reverseTime.reset();
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