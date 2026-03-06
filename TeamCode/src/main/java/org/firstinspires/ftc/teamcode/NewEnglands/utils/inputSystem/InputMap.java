package org.firstinspires.ftc.teamcode.NewEnglands.utils.inputSystem;

import com.qualcomm.robotcore.hardware.Gamepad;

public class InputMap {
    private final Gamepad gamepad;
    private final GamepadInput input;

    public InputMap(GamepadInput input, Gamepad gamepad) {
        this.input = input;
        this.gamepad = gamepad;
    }

    public boolean update() {
        switch (input) {
            case a:
                return gamepad.a;
            case b:
                return gamepad.b;
            case x:
                return gamepad.x;
            case y:
                return gamepad.y;
            case left_bumper:
                return gamepad.left_bumper;
            case right_bumper:
                return gamepad.right_bumper;
            case right_trigger:
                if (gamepad.right_trigger > 0.3) {
                    return true;
                } else return false;
            case left_trigger:
                if (gamepad.left_trigger > 0.3) {
                    return true;
                } else return false;
            default:
                return false;
        }
    }

}
