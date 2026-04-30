package org.firstinspires.ftc.teamcode.nePremier.utils.gamepadSystem;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.nePremier.utils.zinputSystem.DriverMap;
import org.firstinspires.ftc.teamcode.nePremier.utils.zinputSystem.WeNeeeeedToGetGoooder;


public final class Control {
    private final ControlType type;
    private final InputMap map;
    private Command[] commandArray;
    private WeNeeeeedToGetGoooder state = WeNeeeeedToGetGoooder.OFF;
    private boolean active = false;
    private boolean released = true;
    private boolean mapped = false;
    private int mapKey = 0;
    private double scale = 1;

    /// Constructor for input based control types (gamepad activated)
    public Control (GamepadInput input, Gamepad gamepad, ControlType type, Command... command) throws IllegalArgumentException {
        if (type != ControlType.Hold & type != ControlType.Toggle) {
            throw new IllegalArgumentException("Control type must be Auto or Continuous otherwise inputs must be specified");
        }

        this.type = type;
        this.map = new InputMap(input, gamepad);

        constructArray(command);
    }

    /// Constructor for input based control types with scaling for linear type (gamepad activated)
    public Control (GamepadInput input, Gamepad gamepad, ControlType type, double scale, Command... command) throws IllegalArgumentException {
        if (type != ControlType.Linear) {
            throw new IllegalArgumentException("Scale should not be specified for non-linear controls");
        }

        this.scale = scale;
        this.type = type;
        this.map = new InputMap(input, gamepad);

        constructArray(command);
    }

   /// Constructor for input based control types with map functionality (gamepad activated)
    public Control (GamepadInput input, Gamepad gamepad, ControlType type, int map, Command... command) throws IllegalArgumentException {
        if (type != ControlType.Hold & type != ControlType.Toggle) {
            throw new IllegalArgumentException("Control type must be Toggle or Hold otherwise controls should not be specified");
        }

        activateMap();
        this.mapKey = map;
        this.type = type;
        this.map = new InputMap(input, gamepad);

        constructArray(command);
    }

    /// Constructor for input based control types with scaling for linear type and map functionality (gamepad activated)
    public Control (GamepadInput input, Gamepad gamepad, ControlType type, double scale, int map, Command... command) throws IllegalArgumentException {
        if (type != ControlType.Linear) {
            throw new IllegalArgumentException("Scale should not be specified for non-linear controls");
        }

        activateMap();
        this.scale = scale;
        this.type = type;
        this.map = new InputMap(input, gamepad);

        constructArray(command);
    }


    /// Constructor for auto control-type (code activated)
    public Control (ControlType type, Command... command) throws IllegalArgumentException {
        if (type != ControlType.Auto) {
            throw new IllegalArgumentException("Control type must be Toggle or Hold otherwise controls should not be specified");
        }

        this.type = type;
        this.map = null;

        constructArray(command);
    }

    private void constructArray(Command... command) {
        commandArray = command;
    }
    private int getMap(){
        return DriverMap.getMap();
    }

    private void updateScheduler() {
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
    private void runSet() {
        WeNeeeeedToGetGoooder currentState = state;
        for (Command command : commandArray) {
            command.schedule();
        }
        state = WeNeeeeedToGetGoooder.LOOPING;
    }
    private void stopSet() {
        for (Command command : commandArray) {
            command.cancel();
        }
        state = WeNeeeeedToGetGoooder.OFF;
    }

    public void update() {
        if (!mapped) updateScheduler();
        else if (getMap() == mapKey) updateScheduler();
        else stopSet();
    }

    public void run() {
        active = true;
    }
    public void stop() {
        active = false;
    }
    public void activateMap() {
        mapped = true;
    }
    public boolean isRunning() {
        return active;
    }

}
