package org.firstinspires.ftc.teamcode.NewEnglands.robot.commands;

import org.firstinspires.ftc.teamcode.NewEnglands.robot.systems.Transfer;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.BaseCommand;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.CommandLoop;

public class Outake extends BaseCommand {
    private final Transfer transfer;
    public Outake(CommandLoop maps, Transfer transfer) {
        super(maps);
        addRequirement(transfer);
        this.transfer = transfer;
    }

    @Override
    public void init() {
        transfer.reverse();
    }

    @Override
    public void stop() {
        transfer.stop();
    }

}


