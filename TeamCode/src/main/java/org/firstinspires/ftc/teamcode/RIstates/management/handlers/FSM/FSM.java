package org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM;

import org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.states.AllianceSelectorState;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.states.BackoutState;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.states.IntakeState;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.states.NormState;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.states.ShootState;
import org.firstinspires.ftc.teamcode.RIstates.management.SystemManager;

import java.util.EnumMap;


public class FSM {
    public enum StateName {
        Shoot,
        Intake,
        Backout,
        //FollowerError,
        AllianceSelect,
        Norm
    }
    private final EnumMap<StateName, State> stateMap;
    private State currentState = null;

    private final SystemManager manager;



    public FSM(SystemManager manager) {
        this.manager = manager;
        stateMap = new EnumMap<>(StateName.class);
        stateMap.put(StateName.Norm, new NormState());
        stateMap.put(StateName.Shoot, new ShootState());
        stateMap.put(StateName.Intake, new IntakeState());
        stateMap.put(StateName.Backout, new BackoutState());
        stateMap.put(StateName.AllianceSelect, new AllianceSelectorState());
        //stateMap.put(StateName.FollowerError, new FollowerErrorState());
    }

    public void update() {
        if (currentState != null) {
            currentState.update(manager, this);
            manager.telemetryM.addData("Current State", currentState.getClass().getSimpleName());
        }
    }

    public void runNew(StateName newState) {
        if (currentState != null && newState != null && manager != null) {
            if (currentState.equals(newState)) {
                return;
            }

            currentState.end(manager);

            currentState = stateMap.get(newState);

            assert currentState != null;
            currentState.initiate(manager);
        }
    }


}
