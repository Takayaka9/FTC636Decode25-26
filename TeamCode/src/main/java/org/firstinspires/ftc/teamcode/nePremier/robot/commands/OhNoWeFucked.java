package org.firstinspires.ftc.teamcode.nePremier.robot.commands;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.nePremier.robot.systems.Turret;
import org.firstinspires.ftc.teamcode.nePremier.robot.systems.turret.TurretI;
import org.firstinspires.ftc.teamcode.nePremier.utils.commandUtils.BaseCommand;

public class OhNoWeFucked extends BaseCommand {
    final Turret turret;
    Gamepad gamepad = null;
    int ticks = 0;
    public OhNoWeFucked(Turret turret, Gamepad gamepad) {
        super();
        this.turret = turret;
        if (gamepad != null) {
            this.gamepad = gamepad;
        }
    }

    @Override
    public void loop() {
        ticks = ticks + Math.round(gamepad.left_stick_x * 40);
        turret.turnTurret(ticks);
    }
    @Override
    public void stop() {
//        turret.resetEncoder();
    }
}
