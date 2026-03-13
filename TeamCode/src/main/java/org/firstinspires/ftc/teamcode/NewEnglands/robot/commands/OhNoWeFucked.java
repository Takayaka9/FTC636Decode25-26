package org.firstinspires.ftc.teamcode.NewEnglands.robot.commands;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.NewEnglands.robot.systems.Turret;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.BaseCommand;

public class OhNoWeFucked extends BaseCommand {
    Turret turret;
    Gamepad gamepad;
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
