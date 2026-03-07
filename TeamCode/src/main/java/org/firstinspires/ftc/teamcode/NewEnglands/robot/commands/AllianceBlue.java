package org.firstinspires.ftc.teamcode.NewEnglands.robot.commands;

import org.firstinspires.ftc.teamcode.NewEnglands.utils.alliance.Alliance;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.alliance.CurrentAlliance;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.BaseCommand;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.CommandLoop;

public class AllianceBlue extends BaseCommand {
    public AllianceBlue(CommandLoop maps) {
        super(maps);
    }

    @Override
    public void init(){
        CurrentAlliance.alliance = Alliance.BLUE;
    }
}
