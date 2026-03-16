package org.firstinspires.ftc.teamcode.nePremier.robot.commands;

import org.firstinspires.ftc.teamcode.nePremier.robot.systems.servos.CrLiftServo;
import org.firstinspires.ftc.teamcode.nePremier.utils.commandUtils.BaseCommand;
import org.firstinspires.ftc.teamcode.nePremier.utils.commandUtils.CommandLoop;

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
