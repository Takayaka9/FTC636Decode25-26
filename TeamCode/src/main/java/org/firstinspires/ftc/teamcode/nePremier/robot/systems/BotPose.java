package org.firstinspires.ftc.teamcode.nePremier.robot.systems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.Alliance;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.CurrentAlliance;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.LocalizationHelper;
import org.firstinspires.ftc.teamcode.nePremier.utils.commandUtils.BaseSubsystem;

public class BotPose extends BaseSubsystem {
    Follower follower;
    InterpLUT flightTime = new InterpLUT();
    public BotPose(Follower follower) {
        super();
        this.follower = follower;
        flightTime.add(0, 0);
        flightTime.add(0, 0);
        flightTime.add(0, 0);
        flightTime.createLUT();
    }
    public Pose getBotPose(){
        Alliance alliance = CurrentAlliance.alliance;

        double x = follower.getPose().getX();
        double y = follower.getPose().getY();

        double velX = follower.getVelocity().getXComponent() * flightTime.get(LocalizationHelper.getTargetDistance(follower.getPose(), alliance));
        double velY = follower.getVelocity().getYComponent() * flightTime.get(LocalizationHelper.getTargetDistance(follower.getPose(), alliance));

        double accelX = follower.getAcceleration().getXComponent() * flightTime.get(LocalizationHelper.getTargetDistance(follower.getPose(), alliance));
        double accelY = follower.getAcceleration().getYComponent() * flightTime.get(LocalizationHelper.getTargetDistance(follower.getPose(), alliance));

        double newX = x + velX + accelX;
        double newY = y + velY + accelY;

        return new Pose(newX, newY, follower.getHeading());
    }
}
