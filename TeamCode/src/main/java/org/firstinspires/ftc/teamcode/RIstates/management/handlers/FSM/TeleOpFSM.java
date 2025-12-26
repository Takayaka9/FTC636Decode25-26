package org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM;

import org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.states.BackoutState;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.states.IntakeState;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.states.NormState;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.states.ShootState;
import org.firstinspires.ftc.teamcode.RIstates.management.SystemManager;

import java.util.EnumMap;


public class TeleOpFSM {
    public enum StateName {
        Shoot,
        Intake,
        Backout,
        //FollowerError,
        Norm
    }
    private final EnumMap<StateName, State> stateMap;
    private State currentState = null;

    private final SystemManager manager;

    public TeleOpFSM(SystemManager manager) {
        this.manager = manager;
        stateMap = new EnumMap<>(StateName.class);
        stateMap.put(StateName.Norm, new NormState());
        stateMap.put(StateName.Shoot, new ShootState());
        stateMap.put(StateName.Intake, new IntakeState());
        stateMap.put(StateName.Backout, new BackoutState());
        //stateMap.put(StateName.FollowerError, new FollowerErrorState());


    }

    public void update() {
        if (currentState != null) {
            currentState.update(manager, this);
        }

    }

    public void runNew(StateName newState) {
        if (currentState.equals(newState)) {
            return;
        }

        if (currentState != null) {
            currentState.end(manager);
        }

        // Switch to the new state object from the map.
        currentState = stateMap.get(newState);

        // Call the setup method of the new state we are ENTERING.
        currentState.initiate(manager);
    }


}
