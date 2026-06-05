package org.firstinspires.ftc.teamcode.nePremier.robot.commands;

import static org.firstinspires.ftc.teamcode.nePremier.robot.commands.OhNoWeFucked.RADCONFIG.turretLimit;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.nePremier.robot.systems.Turret;
import org.firstinspires.ftc.teamcode.nePremier.utils.commandUtils.BaseCommand;

public class OhNoWeFucked extends BaseCommand {
    private final Turret turret;
    private Gamepad gamepad = null;
    private double radians = 0;
    public OhNoWeFucked(Turret turret, Gamepad gamepad) {
        super();
        this.turret = turret;
        if (gamepad != null) {
            this.gamepad = gamepad;
        }
    }
    @Configurable
    public static class RADCONFIG {
        public static double turretSensitivity = 0.03;
        public static int turretLimit = 60;

    }

    @Override
    public void loop() {
        radians = radians - gamepad.left_stick_x * RADCONFIG.turretSensitivity;
        radians = Math.min(radians, Math.toRadians(turretLimit));
        radians = Math.max(radians, -Math.toRadians(turretLimit));
        turret.turnTurretRad(radians);
    }
    @Override
    public void stop() {
//        turret.resetEncoder();
    }
}
