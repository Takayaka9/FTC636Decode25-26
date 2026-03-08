package org.firstinspires.ftc.teamcode.NewEnglands.robot.commands;

import org.firstinspires.ftc.teamcode.NewEnglands.robot.systems.Transfer;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.BaseCommand;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.Command;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.CommandLoop;

public class Intake extends BaseCommand {
    private final Transfer transfer;
    @Command
    public Intake(CommandLoop maps, Transfer transfer) {
        super();
        addRequirement(transfer);
        this.transfer = transfer;
    }

    @Override
    public void init() {
        transfer.run();
    }

    @Override
    public void stop() {
        transfer.stop();
    }

}


