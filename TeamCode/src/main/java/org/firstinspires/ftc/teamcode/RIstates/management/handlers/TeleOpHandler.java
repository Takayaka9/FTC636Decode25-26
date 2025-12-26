package org.firstinspires.ftc.teamcode.RIstates.management.handlers;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.FSM;

public class TeleOpHandler {
    private final FSM fsm;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;

    public TeleOpHandler(FSM fsm, Gamepad gamepad1, Gamepad gamepad2) {
        this.fsm = fsm;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }


    private boolean changedA = false;
    public boolean changedB = false;
    public boolean changedX = false;
    private boolean changedRT = false;
    private boolean changedLT = false;
    private boolean allianceSelecting = false;


    public void update() {
        if (gamepad2.right_trigger > 0.3 && gamepad2.left_trigger > 0.3 && !allianceSelecting) {
            fsm.runNew(FSM.StateName.AllianceSelect);
            allianceSelecting = true;
        } else {
            allianceSelecting = false;
        }

        if (gamepad2.a && !changedA) {
            changedA = true;
            fsm.runNew(FSM.StateName.Shoot);
        }

        if (gamepad2.right_bumper && !changedRT) {
            changedRT = true;
            fsm.runNew(FSM.StateName.Intake);
        }

        if (gamepad2.left_bumper && !changedLT) {
            changedLT = true;
            fsm.runNew(FSM.StateName.Backout);
        }

        else if (!changedA && !allianceSelecting) {
            fsm.runNew(FSM.StateName.Norm);
        }

        if (!changedA && !changedB && !changedLT && !changedRT && !allianceSelecting) {
            fsm.update();
            changedA = false;
            changedRT = false;
            changedLT = false;
        }

    }
}

