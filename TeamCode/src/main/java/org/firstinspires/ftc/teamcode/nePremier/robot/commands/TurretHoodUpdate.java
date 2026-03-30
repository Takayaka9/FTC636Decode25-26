package org.firstinspires.ftc.teamcode.nePremier.robot.commands;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.nePremier.robot.systems.Turret;
import org.firstinspires.ftc.teamcode.nePremier.robot.systems.servos.HoodServo;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.CurrentAlliance;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.LocalizationHelper;
import org.firstinspires.ftc.teamcode.nePremier.utils.commandUtils.BaseCommand;

@Deprecated
public class TurretHoodUpdate extends BaseCommand {
    private final Turret turret;
    private final HoodServo hood;
    private final Follower follower;
    private Gamepad gamepad = null;
    private boolean overridden = false;

    public TurretHoodUpdate(Turret turret, HoodServo hood, Follower follower, Gamepad gamepad) {
        super();
        this.turret = turret;
        this.hood = hood;
        this.follower = follower;
        if (gamepad != null) {
            this.gamepad = gamepad;
        }
    }

    @Override
    public void loop() {
        if (gamepad != null) {
            if (gamepad.x) {
                double overPos = turret.turretPosition() + gamepad.left_stick_x * Turret.TurretConstants.overrideSensitivity;
                turret.turnTurret(Math.round(overPos));
                overridden = true;
            } else if (overridden) {
                turret.resetEncoder();
                overridden = false;
            } else {
                turret.trackGoal();
            }
        } else {
            turret.trackGoal();
        }
        hood.angleHood(LocalizationHelper.getTargetDistance(follower.getPose(), CurrentAlliance.alliance));
    }

}
