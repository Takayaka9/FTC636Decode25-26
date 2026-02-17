package org.firstinspires.ftc.teamcode.robot.commands;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.robot.systems.Turret;
import org.firstinspires.ftc.teamcode.robot.systems.servos.HoodServo;
import org.firstinspires.ftc.teamcode.utils.alliance.CurrentAlliance;
import org.firstinspires.ftc.teamcode.utils.alliance.GetTargetDistance;
import org.firstinspires.ftc.teamcode.utils.commandUtils.BaseCommand;
import org.firstinspires.ftc.teamcode.utils.commandUtils.CommandLoop;

public class TurretHoodUpdate extends BaseCommand {
    private final Turret turret;
    private final HoodServo hood;
    private final GetTargetDistance getTargetDistance;
    private final Follower follower;

    public TurretHoodUpdate(CommandLoop maps, Turret turret, HoodServo hood, Follower follower) {
        super(maps);
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
