package org.firstinspires.ftc.teamcode.NewEnglands.robot.commands;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.NewEnglands.robot.systems.Turret;
import org.firstinspires.ftc.teamcode.NewEnglands.robot.systems.servos.HoodServo;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.alliance.CurrentAlliance;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.alliance.GetTargetDistance;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.BaseCommand;

public class TurretHoodUpdate extends BaseCommand {
    private final Turret turret;
    private final HoodServo hood;
    private final GetTargetDistance getTargetDistance;
    private final Follower follower;
    private Gamepad gamepad;
    private boolean overridden = false;

    public TurretHoodUpdate(Turret turret, HoodServo hood, Follower follower, Gamepad gamepad) {
        super();
        addRequirement(turret, hood);
        this.turret = turret;
        this.hood = hood;
        this.getTargetDistance = new GetTargetDistance();
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
        hood.angleHood(getTargetDistance.getTargetDistance(follower.getPose(), CurrentAlliance.alliance));
    }

}
