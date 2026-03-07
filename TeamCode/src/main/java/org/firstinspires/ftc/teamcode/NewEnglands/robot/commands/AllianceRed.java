package org.firstinspires.ftc.teamcode.NewEnglands.robot.commands;

import org.firstinspires.ftc.teamcode.NewEnglands.utils.alliance.Alliance;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.alliance.CurrentAlliance;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.BaseCommand;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.CommandLoop;

public class AllianceRed extends BaseCommand {
    public AllianceRed(CommandLoop maps) {
        super(maps);
    }

    @Override
    public void init(){
        CurrentAlliance.alliance = Alliance.RED;
    }
}
