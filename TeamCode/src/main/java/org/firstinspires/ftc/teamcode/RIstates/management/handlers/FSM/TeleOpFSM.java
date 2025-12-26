package org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.states.AllianceSelectorState;
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
        AllianceSelect,
        Norm
    }
    private final EnumMap<StateName, State> stateMap;
    private State currentState = null;

    private final SystemManager manager;
    private final Gamepad gamepad;



    public TeleOpFSM(SystemManager manager, Gamepad gamepad) {
        this.manager = manager;
        this.gamepad = gamepad;

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
            currentState.update(manager, this, gamepad);
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
