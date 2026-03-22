package org.firstinspires.ftc.teamcode.nePremier.opModes.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.nePremier.utils.init.Initializer;

@TeleOp
public class ServoZero extends OpMode {
    public enum servoSelect {
        hood,
        stopper,
        none
    }

    public static class servoTestParams {
        public static servoSelect pickServo = servoSelect.none;
        public static double position = 0;
    }

    Initializer i = null;

    @Override
    public void init() {
        i = new Initializer(gamepad1, gamepad2, hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        switch (servoTestParams.pickServo) {
            case hood:
                i.hoodServo.setPosition(servoTestParams.position);
                break;
            case stopper:
                i.stopper.setPosition(servoTestParams.position);
                break;
            case none:
                break;
        }
    }
}
