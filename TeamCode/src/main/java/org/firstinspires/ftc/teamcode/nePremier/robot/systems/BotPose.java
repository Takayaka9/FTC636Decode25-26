package org.firstinspires.ftc.teamcode.nePremier.robot.systems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.Alliance;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.CurrentAlliance;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.LocalizationHelper;
import org.firstinspires.ftc.teamcode.nePremier.utils.commandUtils.BaseSubsystem;

@Configurable
public class BotPose extends BaseSubsystem {
    Follower follower;
    InterpLUT flightTime = new InterpLUT();
    public BotPose(Follower follower) {
        super();
        this.follower = follower;
        flightTime.add(0, 0.22); //tuned, 0.22
        flightTime.add(36, 0.22); //tuned, 0.22
        flightTime.add(53.6, 0.14); //tuned, 0.14
//        flightTime.add(73.5, );
//        flightTime.add(100, );
//        flightTime.add( , );
//        flightTime.add( , );
        flightTime.add(1000, 5); //tuned
        flightTime.createLUT();
    }
    public Pose getBotPose(){
        Alliance alliance = CurrentAlliance.alliance;

        double x = follower.getPose().getX();
        double y = follower.getPose().getY();

        double velX = follower.getVelocity().getXComponent() * flightTime.get(LocalizationHelper.getTargetDistance(follower.getPose(), alliance));
        double velY = follower.getVelocity().getYComponent() * flightTime.get(LocalizationHelper.getTargetDistance(follower.getPose(), alliance));

        double accelX = 0.5 * follower.getAcceleration().getXComponent() * Math.pow(flightTime.get(LocalizationHelper.getTargetDistance(follower.getPose(), alliance)), 2);
        double accelY = 0.5 * follower.getAcceleration().getYComponent() * Math.pow(flightTime.get(LocalizationHelper.getTargetDistance(follower.getPose(), alliance)), 2);

        double newX = x + velX + accelX;
        double newY = y + velY + accelY;

        return new Pose(newX, newY, follower.getHeading());
    }
}
