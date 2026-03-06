package org.firstinspires.ftc.teamcode.NewEnglands.utils.inputSystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.BaseCommand;

import java.util.HashSet;

abstract class LogicBase {
    private final ControlType type;
    private final Gamepad gamepad;
    private final HashSet<BaseCommand> commandSet;
    private boolean active;

    public LogicBase(ControlType type, Gamepad gamepad, BaseCommand... command) {
        this.type = type;
        this.gamepad = gamepad;
        commandSet = new HashSet<>();
        for (int i = 0; i < command.length; i++) {
            commandSet.add(command[i]);
        }

    }

    public void runSet() {

    }

    public void update() throws Exception {
        switch (type) {
            case Toggle:

                break;
            case Hold:
                if (gamepad.)
                break;
            case Linear:
                throw new Exception("Linear control type not implemented aka taka did something bad");
        }
    }


}
