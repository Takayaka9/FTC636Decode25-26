package org.firstinspires.ftc.teamcode.robot.commands;

import org.firstinspires.ftc.teamcode.robot.systems.Transfer;
import org.firstinspires.ftc.teamcode.utils.commandUtils.BaseCommand;
import org.firstinspires.ftc.teamcode.utils.commandUtils.BaseSubsystem;
import org.firstinspires.ftc.teamcode.utils.commandUtils.CommandLoop;

public class Intake extends BaseCommand {
    private final Transfer transfer;
    public Intake(CommandLoop maps, Transfer transfer) {
        super(maps);
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


