package org.firstinspires.ftc.teamcode.NewEnglands.robot.commands;

import org.firstinspires.ftc.teamcode.NewEnglands.robot.systems.Transfer;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.BaseCommand;

public class Intake extends BaseCommand {
    private final Transfer transfer;
    public Intake(Transfer transfer) {
        super();
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


