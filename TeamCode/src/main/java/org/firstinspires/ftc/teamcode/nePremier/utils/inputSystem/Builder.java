package org.firstinspires.ftc.teamcode.nePremier.utils.inputSystem;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.nePremier.utils.commandUtils.BaseCommand;

import java.util.ArrayList;
import java.util.Arrays;

public abstract class Builder {

    //declarations and initial default values
    GamepadInput input = null;
    Gamepad gamepad = null;
    ControlType type = ControlType.Hold;
    double scale = 1;
    BaseCommand[] commands = null;
    ArrayList<Control> controls = new ArrayList<>();

    ///reset to default values once a control has been added
    private void reset() {
        input = null;
        gamepad = null;
        type = null;
        scale = 1;
        commands = null;
    }

    /// Check to make sure the values for Control constructor have been added
    private void checkExceptions() {
        if (input == null & gamepad == null & commands == null) {
            throw new IllegalArgumentException(
                    "Required values not entered, make sure to add an input, a gamepad, and at least 1 command"
            );
        }
    }

    /// Method to add a control
    public void addControl() throws IllegalArgumentException {
        switch (type) {
            case Auto:
                throw new IllegalArgumentException("Use ivy instead");
            case Hold:
            case Toggle:
                controls.add(new Control(input, gamepad, type, commands));
                break;
            case Linear:
                controls.add(new Control(input, gamepad, type, scale, commands));
                break;
        }
        reset();
    }

    /// Method to add a control, method inputs instead of separate methods to set parameters
    public void addControl(Control control) throws IllegalArgumentException {
        controls.add(control);
        reset();
    }

    /// Method to add a control, method inputs instead of separate methods to set parameters
    public void addControl(Control... control) throws IllegalArgumentException {
        controls.addAll(Arrays.asList(control));
        reset();
    }


    public void setInput(GamepadInput input) {
        this.input = input;
    }
    public void setGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;
    }
    public void setType(ControlType type) {
        this.type = type;
    }
    public void setScale(double scale) {
        this.scale = scale;
    }
    public void setControls(BaseCommand... commands) {
        this.commands = commands;
    }

}
