package org.firstinspires.ftc.teamcode.NewEnglands.robot.commands;

import org.firstinspires.ftc.teamcode.NewEnglands.robot.systems.servos.CrLiftServo;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.BaseCommand;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.CommandLoop;

@Deprecated
public class CrLiftBot extends BaseCommand {
    private final CrLiftServo lift;
    public CrLiftBot(CommandLoop maps, CrLiftServo lift) {
        super();
        this.lift = lift;
    }

    @Override
    public void init() {
        lift.up();
    }
}
