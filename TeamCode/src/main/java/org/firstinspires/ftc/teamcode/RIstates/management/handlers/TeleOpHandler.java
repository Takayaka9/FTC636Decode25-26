package org.firstinspires.ftc.teamcode.RIstates.management.handlers;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RIstates.management.fsm.FSM;

public class TeleOpHandler {
    private final FSM fsm;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    private final ShooterHandler shooterHandler;

    public TeleOpHandler(FSM fsm, Gamepad gamepad1, Gamepad gamepad2, ShooterHandler shooterHandler) {
        this.fsm = fsm;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.shooterHandler = shooterHandler;
    }

    private boolean changedA = false;
    public boolean changedB = false;
    public boolean changedX = false;
    private boolean changedRT = false;
    private boolean changedLT = false;
    private boolean allianceSelecting = false;
    private boolean off = false;

    private FSM.StateName requestingTransition = null;
    public void setTransition(FSM.StateName stateName){
            requestingTransition = stateName;
    }



    public void start(){
        fsm.runNew(FSM.StateName.Norm);
    }
    public void stop(){
        fsm.runNew(FSM.StateName.Norm);
    }

    public void update() {
        fsm.update();
        /*
        if statement order determines priority
        logic concept
        if inputs && not already active then request transition
        if no inputs && already active then stop state
         */
        if (gamepad2.left_trigger > 0.3 && gamepad2.right_trigger > 0.3 && !allianceSelecting) {
            setTransition(FSM.StateName.AllianceSelect);
            allianceSelecting = true;
            off = false;
        } else if (gamepad2.left_trigger < 0.3 & gamepad2.right_trigger < 0.3 && allianceSelecting) {
            allianceSelecting = false;
            off = true;
        }

        //different logic which checks shootRunning for stop
        if (gamepad2.a && !changedA) {
            changedA = true;
            setTransition(FSM.StateName.Shoot);
            off = false;
        } else if (shooterHandler.shooterRunning && changedA) {
            changedA = false;
            off = true;
        }

        //backout
        if (gamepad2.left_bumper && !changedLT) {
            changedLT = true;
            setTransition(FSM.StateName.Backout);
            off = false;
        } else if (!gamepad2.left_bumper && changedLT) {
            changedLT = false;
            off = true;
        }

        //intake
        if (gamepad2.right_bumper && !changedRT) {
            changedRT = true;
            setTransition(FSM.StateName.Intake);
            off = false;
        } else if (!gamepad2.right_bumper && changedRT) {
            changedRT = false;
            off = true;
        }

//        transition to next state
        if (off) {
            setTransition(FSM.StateName.Norm);
        }

        if (requestingTransition != null) {
            fsm.runNew(requestingTransition);
        }
//        fsm.runNew(requestingTransition);
//        if (!checkStillRunning()) {
//            setTransition(FSM.StateName.Norm);
//        }

    }
}

