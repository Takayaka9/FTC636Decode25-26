package org.firstinspires.ftc.teamcode.nePremier.robot.commands;

import org.firstinspires.ftc.teamcode.nePremier.robot.systems.Turret;
import org.firstinspires.ftc.teamcode.nePremier.robot.systems.turret.TurretI;
import org.firstinspires.ftc.teamcode.nePremier.utils.commandUtils.BaseCommand;

public class SideWaysTurret extends BaseCommand {
    final Turret turret;
    public SideWaysTurret (Turret turret) {
        this.turret = turret;
    }

    @Override
    public void loop() {
        turret.turnTurretRad(Math.toRadians(85));
    }
}
