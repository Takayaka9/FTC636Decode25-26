package org.firstinspires.ftc.teamcode.nePremier.robot.commands;

import org.firstinspires.ftc.teamcode.nePremier.robot.systems.servos.LiftServo;
import org.firstinspires.ftc.teamcode.nePremier.utils.commandUtils.BaseCommand;

@Deprecated
public class LiftBot extends BaseCommand {
    private final LiftServo lift;
    public LiftBot(LiftServo lift) {
        super();
        this.lift = lift;
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
