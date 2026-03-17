package org.firstinspires.ftc.teamcode.nePremier.robot.commands;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.nePremier.robot.systems.Turret;
import org.firstinspires.ftc.teamcode.nePremier.utils.commandUtils.BaseCommand;

public class OhNoWeFucked extends BaseCommand {
    final Turret turret;
    Gamepad gamepad = null;
    public OhNoWeFucked(Turret turret, Gamepad gamepad) {
        super();
        this.turret = turret;
        if (gamepad != null) {
            this.gamepad = gamepad;
        }
    }

    @Override
    public void loop() {
        turret.turnTurret(Math.round(gamepad.left_stick_x * 100));
    }
    @Override
    public void stop() {
        turret.resetEncoder();
    }
}
