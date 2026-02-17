package org.firstinspires.ftc.teamcode.robot.commands;

import org.firstinspires.ftc.teamcode.robot.systems.servos.LiftServo;
import org.firstinspires.ftc.teamcode.utils.commandUtils.BaseCommand;
import org.firstinspires.ftc.teamcode.utils.commandUtils.CommandLoop;

public class LiftBot extends BaseCommand {
    LiftServo lift;
    public LiftBot (CommandLoop maps, LiftServo lift) {
        super(maps);
        this.lift = lift;
        addRequirement(lift);
    }

    @Override
    public void loop() {
        lift.up();
    }
}
