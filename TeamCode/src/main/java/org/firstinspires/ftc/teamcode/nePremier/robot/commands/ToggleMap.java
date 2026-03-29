package org.firstinspires.ftc.teamcode.nePremier.robot.commands;

import org.firstinspires.ftc.teamcode.nePremier.utils.commandUtils.BaseCommand;
import org.firstinspires.ftc.teamcode.nePremier.utils.inputSystem.DriverMap;

public class ToggleMap extends BaseCommand {

    public ToggleMap() {
        super();
        DriverMap.setMap(0);
    }

    @Override
    public void init() {
        DriverMap.setMap(1);
    }

    @Override
    public void stop() {
        DriverMap.setMap(0);
    }

}
