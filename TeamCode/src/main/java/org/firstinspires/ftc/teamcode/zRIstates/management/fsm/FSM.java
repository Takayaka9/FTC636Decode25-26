package org.firstinspires.ftc.teamcode.zRIstates.management.fsm;

import org.firstinspires.ftc.teamcode.zRIstates.management.fsm.states.AllianceSelectorState;
import org.firstinspires.ftc.teamcode.zRIstates.management.fsm.states.AutoShootState;
import org.firstinspires.ftc.teamcode.zRIstates.management.fsm.states.BackoutState;
import org.firstinspires.ftc.teamcode.zRIstates.management.fsm.states.IntakeState;
import org.firstinspires.ftc.teamcode.zRIstates.management.fsm.states.NormState;
import org.firstinspires.ftc.teamcode.zRIstates.management.fsm.states.ShootState;
import org.firstinspires.ftc.teamcode.zRIstates.management.SystemManager;

import java.util.EnumMap;


public class FSM {
    public enum StateName {
        Shoot,
        Intake,
        Backout,
        //FollowerError,
        AllianceSelect,
        Norm,
        AutoShoot
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
        stateMap.put(StateName.AutoShoot, new AutoShootState());
    }

    public String getCurrentStateAsString(){
        if (currentState != null) {
            return currentState.getClass().getSimpleName();
        }
        return "null";
    }

    public StateName getCurrentStateName(){
        if (currentState != null) {
            if (getCurrentStateAsString().equals("ShootState")) {
                return StateName.Shoot;
            }
            if (getCurrentStateAsString().equals("IntakeState")) {
                return StateName.Intake;
            }
            if (getCurrentStateAsString().equals("BackoutState")) {
                return StateName.Backout;
            }
            if (getCurrentStateAsString().equals("AllianceSelectorState")) {
                return StateName.AllianceSelect;
            }
            if(getCurrentStateAsString().equals("AutoShootState")){
                return StateName.AutoShoot;
            }
        }
        return StateName.Norm;
    }


    public void update() {
        if (currentState != null) {
            currentState.update(manager, this);
            manager.telemetryM.addData("Current State", currentState.getClass().getSimpleName());
        }
    }

    public void runNew(StateName newState) {
        if (newState != null && manager != null) {
            if (currentState != null) {
                currentState.end(manager);
            }

            currentState = stateMap.get(newState);

            assert currentState != null;
            currentState.initiate(manager);
        }
    }


}
