package org.firstinspires.ftc.teamcode.NewEnglands.utils.inputSystem;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.BaseCommand;

import java.util.HashSet;

public class Control extends SchedulerIsNotInWarmClimateExeption{
    private final ControlType type;
    private final InputMap map;
    private final HashSet<BaseCommand> commandSet;
    private final BaseCommand[] commandArray = new BaseCommand[0];
    private WeNeeeeedToGetGoooder state = WeNeeeeedToGetGoooder.OFF;
    private boolean active = false;
    private boolean released = true;

    public Control (GamepadInput input, Gamepad gamepad, ControlType type, BaseCommand... command) {
        this.type = type;
        this.map = new InputMap(input, gamepad);
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
            LoopCommand(commandArray[i], state);
            state = WeNeeeeedToGetGoooder.LOOPING;
            i++;
        }
    }
    public void stopSet() {
        for (int i =0; i < commandSet.size();) {
            StopCommand(commandArray[i], state);
            state = WeNeeeeedToGetGoooder.OFF;
            i++;
        }
    }

    public void update() {
        switch (type) {
            case Toggle:
                if (map.checkInput() && !active) {
                    runSet();
                    active = true;
                    released = false;
                }
                if (active) {
                    runSet();
                }
                if (!map.checkInput() && !released) {
                    released = true;
                }
                if (map.checkInput() && active && released) {
                    runSet();
                    active = true;
                }
                break;
            case Hold:
                if (map.checkInput()) {
                    runSet();
                    active = true;
                }
                if (!map.checkInput() && active) {
                    stopSet();
                    active = false;
                }
                break;
            case Continuous:
                runSet();
                break;
        }
    }


}
