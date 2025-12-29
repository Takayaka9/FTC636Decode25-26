package org.firstinspires.ftc.teamcode.RIstates.management.handlers;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.FSM;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.states.controllers.ShooterController;

public class TeleOpHandler {
    private final FSM fsm;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    private final ShooterController shooterController;

    public TeleOpHandler(FSM fsm, Gamepad gamepad1, Gamepad gamepad2, ShooterController shooterController) {
        this.fsm = fsm;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.shooterController = shooterController;
    }

    private boolean changedA = false;
    public boolean changedB = false;
    public boolean changedX = false;
    private boolean changedRT = false;
    private boolean changedLT = false;
    private boolean allianceSelecting = false;


    private FSM.StateName requestingTransition = null;
    public void setTransition(FSM.StateName stateName){
        if (requestingTransition != null) {
            requestingTransition = stateName;
        }
    }

    public boolean checkStillRunning() {
        if (changedA & changedB & changedX & changedRT & changedLT & allianceSelecting) {
            return true;
        }
        return false;
    }



    public void start(){
        fsm.runNew(FSM.StateName.Norm);
    }
    public void stop(){
        fsm.runNew(FSM.StateName.Norm);
    }

    public void update() {
        fsm.update();
        requestingTransition = null;

        /*
        if statement order determines priority
        logic concept
        if inputs && not already active then request transition
        if no inputs && already active then stop state
         */
        if (gamepad2.left_trigger > 0.3 && gamepad2.right_trigger > 0.3 && !allianceSelecting) {
            setTransition(FSM.StateName.AllianceSelect);
            allianceSelecting = true;
        } else if (gamepad2.left_trigger < 0.3 & gamepad2.right_trigger < 0.3 && allianceSelecting) {
            allianceSelecting = false;
        }

        //different logic which checks shootRunning for stop
        if (gamepad2.a && !changedA) {
            changedA = true;
            setTransition(FSM.StateName.Shoot);
        } else if (shooterController.shooterRunning && changedA) {
            changedA = false;
        }

        //backout
        if (gamepad2.left_bumper && !changedLT) {
            changedLT = true;
            setTransition(FSM.StateName.Backout);
        } else if (!gamepad2.left_bumper && changedLT) {
            changedLT = false;
        }

        //intake
        if (gamepad2.right_bumper && !changedRT) {
            changedRT = true;
            setTransition(FSM.StateName.Intake);
        } else if (!gamepad2.right_bumper && changedRT) {
            changedRT = false;
        }

        //transition to next state
        if (!checkStillRunning()) {
            setTransition(FSM.StateName.Norm);
        }
        if (requestingTransition != null) {
            fsm.runNew(requestingTransition);
        }
    }
}

