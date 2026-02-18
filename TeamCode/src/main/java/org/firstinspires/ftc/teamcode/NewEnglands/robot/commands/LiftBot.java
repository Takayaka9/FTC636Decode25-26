package org.firstinspires.ftc.teamcode.NewEnglands.robot.commands;

import org.firstinspires.ftc.teamcode.NewEnglands.robot.systems.servos.LiftServo;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.BaseCommand;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.CommandLoop;

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
