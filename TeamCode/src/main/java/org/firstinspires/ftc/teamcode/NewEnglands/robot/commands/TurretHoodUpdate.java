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
                turret.turnTurret(Math.round(gamepad.left_stick_x * 100));
                overridden = true;
            } else if (overridden) {
                turret.resetEncoder();
                overridden = false;
            } else {
                turret.trackGoal(CurrentAlliance.alliance);
            }
        } else {
            turret.trackGoal(CurrentAlliance.alliance);
        }
        hood.angleHood(getTargetDistance.getTargetDistance(follower.getPose(), CurrentAlliance.alliance));
    }

}
