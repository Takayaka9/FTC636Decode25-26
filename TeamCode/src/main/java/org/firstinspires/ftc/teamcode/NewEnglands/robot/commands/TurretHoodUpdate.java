package org.firstinspires.ftc.teamcode.NewEnglands.robot.commands;

import com.pedropathing.follower.Follower;

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

    public TurretHoodUpdate(Turret turret, HoodServo hood, Follower follower) {
        super();
        addRequirement(turret, hood);
        this.turret = turret;
        this.hood = hood;
        this.getTargetDistance = new GetTargetDistance();
        this.follower = follower;
    }

    @Override
    public void loop() {
        turret.trackGoal(CurrentAlliance.alliance);
        hood.angleHood(getTargetDistance.getTargetDistance(follower.getPose(), CurrentAlliance.alliance));
    }

}
