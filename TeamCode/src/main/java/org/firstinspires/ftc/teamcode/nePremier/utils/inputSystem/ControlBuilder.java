package org.firstinspires.ftc.teamcode.nePremier.utils.inputSystem;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.nePremier.utils.commandUtils.BaseCommand;

@Deprecated
public class ControlBuilder {
    GamepadInput input = null;
    Gamepad gamepad = null;
    ControlType type = null;
    BaseCommand[] command = null;
    public Control build() {

        return new Control(input, gamepad, type, command);
    }

    public void addInput(GamepadInput input) {
        this.input = input;
    }

    public void addGamepad() {
        this.gamepad = gamepad;
    }

    public void addType() {

    }


}
