package org.firstinspires.ftc.teamcode.nePremier.robot.commands;

import org.firstinspires.ftc.teamcode.nePremier.robot.systems.servos.NewCRLiftServo;
import org.firstinspires.ftc.teamcode.nePremier.utils.commandUtils.BaseCommand;

public class NewCRLiftDown extends BaseCommand {
    private final NewCRLiftServo servo;
    public NewCRLiftDown(NewCRLiftServo servo) {
        super();
        this.servo = servo;
    }

    @Override
    public void init() {
        servo.backward();
    }

    @Override
    public void stop() {
        servo.stop();
    }
}
