package org.firstinspires.ftc.teamcode.NewEnglands.robot.commands;

import org.firstinspires.ftc.teamcode.NewEnglands.utils.alliance.Alliance;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.alliance.CurrentAlliance;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.BaseCommand;

public class AllianceBlue extends BaseCommand {
    public AllianceBlue() {
        super();
    }

    @Override
    public void init(){
        CurrentAlliance.alliance = Alliance.BLUE;
    }
}
