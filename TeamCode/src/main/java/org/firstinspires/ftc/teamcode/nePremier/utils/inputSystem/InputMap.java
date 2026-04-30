package org.firstinspires.ftc.teamcode.nePremier.utils.inputSystem;

import com.qualcomm.robotcore.hardware.Gamepad;

public class InputMap {
    private final Gamepad g;
    private final GamepadInput input;

    public InputMap(GamepadInput input, Gamepad gamepad) {
        this.input = input;
        this.g = gamepad;
    }

    public boolean checkInput() {
        if (g != null && input != null) {
            switch (input) {
                case a:
                    return g.a;
                case b:
                    return g.b;
                case x:
                    return g.x;
                case y:
                    return g.y;
                case left_bumper:
                    return g.left_bumper;
                case right_bumper:
                    return g.right_bumper;
                case right_trigger:
                    return g.right_trigger > 0.3;
                case left_trigger:
                    return g.left_trigger > 0.3;
                case dpad_down:
                    return g.dpad_down;
                case dpad_left:
                    return g.dpad_left;
                case dpad_right:
                    return g.dpad_right;
                case dpad_up:
                    return g.dpad_up;
                default:
                    return false;
            }
        }
        return false;
    }

}
