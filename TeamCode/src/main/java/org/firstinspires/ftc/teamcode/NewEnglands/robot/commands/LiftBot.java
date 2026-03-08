package org.firstinspires.ftc.teamcode.NewEnglands.robot.commands;

import org.firstinspires.ftc.teamcode.NewEnglands.robot.systems.servos.LiftServo;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.BaseCommand;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.CommandLoop;

public class LiftBot extends BaseCommand {
    private final LiftServo lift;
    public LiftBot(CommandLoop maps, LiftServo lift) {
        super();
        this.lift = lift;
        addRequirement(lift);
        this.lift.up();
    }

    @Override
    public void init() {
        lift.down();
    }

    @Override
    public void stop() {
        lift.up();
    }



}
