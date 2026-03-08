package org.firstinspires.ftc.teamcode.NewEnglands.robot.commands;

import org.firstinspires.ftc.teamcode.NewEnglands.robot.systems.servos.CrLiftServo;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.BaseCommand;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.Command;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.CommandLoop;

public class CrLiftBot extends BaseCommand {
    private final CrLiftServo lift;
    @Command
    public CrLiftBot(CommandLoop maps, CrLiftServo lift) {
        super();
        this.lift = lift;
        addRequirement(lift);
    }

    @Override
    public void init() {
        lift.up();
    }
}
