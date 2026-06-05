package org.firstinspires.ftc.teamcode.nePremier.opModes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.nePremier.robot.systems.servos.NewCRLiftServo;
@Disabled
@Configurable
@TeleOp
public class CRLiftTester extends OpMode {
    NewCRLiftServo servoSystem = null;

    public enum testStates {
        OFF,
        Forward,
        Backward
    }

    public static testStates currentState = testStates.OFF;

    @Override
    public void init() {
        servoSystem = new NewCRLiftServo(hardwareMap);
    }

    @Override
    public void loop() {
        switch(currentState) {
            case Forward:
                servoSystem.forward();
                break;
            case Backward:
                servoSystem.backward();
                break;
            case OFF:
                servoSystem.stop();
                break;
        }
    }
}
