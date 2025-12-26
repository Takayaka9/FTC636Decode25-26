package org.firstinspires.ftc.teamcode.RIstates.management.handlers;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.TeleOpFSM;

public class TeleOpHandler {
    private final TeleOpFSM fsm;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;

    public TeleOpHandler(TeleOpFSM fsm, Gamepad gamepad1, Gamepad gamepad2) {
        this.fsm = fsm;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }


    private boolean changedA = false;
    public boolean changedB = false;
    public boolean changedX = false;
    private boolean changedRT = false;
    private boolean changedLT = false;


    public void update() {
        if (!gamepad2.a && !gamepad2.right_bumper && !gamepad2.left_bumper) {
            fsm.update();
            changedA = false;
            changedRT = false;
            changedLT = false;
        }
        if (gamepad2.a && !changedA) {
            changedA = true;
            fsm.runNew(TeleOpFSM.StateName.Shoot);
        }
        if (gamepad2.right_bumper && !changedRT) {
            changedRT = true;
            fsm.runNew(TeleOpFSM.StateName.Intake);
        }
        if (gamepad2.left_bumper && !changedLT) {
            changedLT = true;
            fsm.runNew(TeleOpFSM.StateName.Backout);
        }
    }
}

