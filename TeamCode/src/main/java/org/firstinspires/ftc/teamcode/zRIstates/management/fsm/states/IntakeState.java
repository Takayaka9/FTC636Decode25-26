package org.firstinspires.ftc.teamcode.zRIstates.management.fsm.states;

import org.firstinspires.ftc.teamcode.zRIstates.management.fsm.State;
import org.firstinspires.ftc.teamcode.zRIstates.management.fsm.FSM;
import org.firstinspires.ftc.teamcode.zRIstates.management.SystemManager;

public class IntakeState implements State {

    @Override
    public void initiate(SystemManager manager) {
        manager.intake.run();
        manager.shooter.brake();
    }

    @Override
    public void update(SystemManager manager, FSM fsm) {
        //manager.shooterHandler.constantShoot();
    }

    @Override
    public void end(SystemManager manager) {
        manager.intake.stop();
        //manager.shooterHandler.off();
    }

}