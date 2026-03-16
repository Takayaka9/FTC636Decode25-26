package org.firstinspires.ftc.teamcode.nePremier.robot.commands;

import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.Alliance;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.CurrentAlliance;
import org.firstinspires.ftc.teamcode.nePremier.utils.commandUtils.BaseCommand;

public class AllianceRed extends BaseCommand {
    public AllianceRed() {
        super();
    }

    @Override
    public void init(){
        CurrentAlliance.alliance = Alliance.RED;
    }
}
