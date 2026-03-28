package org.firstinspires.ftc.teamcode.nePremier.utils.inputSystem;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.nePremier.utils.commandUtils.BaseCommand;

public final class Control extends SchedulerIsNotInWarmClimateExeption{
    private final ControlType type;
    private final InputMap map;
    private BaseCommand[] commandArray;
    private WeNeeeeedToGetGoooder state = WeNeeeeedToGetGoooder.OFF;
    private boolean active = false;
    private boolean released = true;

    /// Constructor for input based control types (gamepad activated)
    public Control (GamepadInput input, Gamepad gamepad, ControlType type, BaseCommand... command) throws IllegalArgumentException {
        if (type != ControlType.Hold & type != ControlType.Toggle) {
            throw new IllegalArgumentException("Control type must be Toggle or Hold otherwise controls should not be specified");
        }

        this.type = type;
        this.map = new InputMap(input, gamepad);

        constructArray(command);
    }

    /// Constructor for auto control-type (code activated)
    public Control (ControlType type, BaseCommand... command) throws IllegalArgumentException {
        if (type != ControlType.Auto) {
            throw new IllegalArgumentException("Control type must be Auto or Continuous otherwise inputs must be specified");
        }

        this.type = type;
        this.map = null;

        constructArray(command);
    }

    private void constructArray(BaseCommand... command) {
        commandArray = command;
    }

    private void runSet() {
        WeNeeeeedToGetGoooder currentState = state;
        for (BaseCommand command : commandArray) {
            LoopCommand(command, currentState);
        }
        state = WeNeeeeedToGetGoooder.LOOPING;
    }
    private void stopSet() {
        for (BaseCommand command : commandArray) {
            StopCommand(command, state);
        }
        state = WeNeeeeedToGetGoooder.OFF;
    }
    public void run() {
        active = true;
    }
    public void stop() {
        active = false;
    }
    public boolean isRunning() {
        return active;
    }


        public void update() {
        switch (type) {
            case Toggle:
                assert map != null;
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
                    active = false;
                }
                break;
            case Hold:
                assert map != null;
                if (map.checkInput()) {
                    runSet();
                    active = true;
                }
                if (!map.checkInput() && active) {
                    stopSet();
                    active = false;
                }
                break;
            case Auto:
                if (active) {
                    runSet();
                } else {
                    stopSet();
                }
                break;
        }
    }


}
