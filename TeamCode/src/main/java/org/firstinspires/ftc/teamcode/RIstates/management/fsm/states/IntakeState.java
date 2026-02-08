package org.firstinspires.ftc.teamcode.RIstates.management.fsm.states;

import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Controller;
import org.firstinspires.ftc.teamcode.RIstates.management.fsm.State;
import org.firstinspires.ftc.teamcode.RIstates.management.fsm.FSM;
import org.firstinspires.ftc.teamcode.RIstates.management.SystemManager;

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