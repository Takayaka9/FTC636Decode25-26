package org.firstinspires.ftc.teamcode.NewEnglands.utils.inputSystem;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.BaseCommand;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.CommandLoop;

import java.util.HashSet;

public class Control extends InputMap{
    private final ControlType type;
    private final HashSet<BaseCommand> commandSet;
    private final BaseCommand[] commandArray = new BaseCommand[0];
    private final CommandLoop loop;
    private boolean active = false;
    private boolean released = true;

    public Control (GamepadInput input, Gamepad gamepad, ControlType type, CommandLoop loop, BaseCommand... command) {
        super(input, gamepad);
        this.type = type;
        this.loop = loop;
        commandSet = new HashSet<>();
        for (int i = 0; i < command.length;) {
            commandSet.add(command[i]);
            i++;
        }
        for (int i = 0; i < command.length;) {
            commandArray[i] = command[i];
            i++;
        }
    }

    public void runSet() {
        for (int i =0; i < commandSet.size();) {
            loop.runCommand(commandArray[i]);
            i++;
        }
    }
    public void stopSet() {
        for (int i =0; i < commandSet.size();) {
            loop.stopCommand(commandArray[i]);
            i++;
        }
    }

    public void update() {
        switch (type) {
            case Toggle:
                if (checkInput() && !active) {
                    runSet();
                    active = true;
                    released = false;
                }
                if (!checkInput() && !released) {
                    released = true;
                }
                if (checkInput() && active && released) {
                    runSet();
                    active = true;
                }
                break;
            case Hold:
                if (checkInput() && !active) {
                    runSet();
                    active = true;
                }
                if (!checkInput() && active) {
                    stopSet();
                    active = false;
                }
                break;
        }
    }


}
