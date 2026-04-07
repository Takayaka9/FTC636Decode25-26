package org.firstinspires.ftc.teamcode.nePremier.robot.systems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.Alliance;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.CurrentAlliance;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.LocalizationHelper;
import org.firstinspires.ftc.teamcode.nePremier.utils.commandUtils.BaseSubsystem;

@Configurable
public class BotPose extends BaseSubsystem {
    Follower follower;
    InterpLUT flightTime = new InterpLUT();

    @SuppressWarnings("FieldMayBeFinal")
    @Configurable
    public static class BotPoseConstants {
        public static double leadMultiplier = 1.5;
    }

    public BotPose(Follower follower, TelemetryManager t) {
        super();
        this.follower = follower;
        flightTime.add(0, 0.22); //tuned, 0.22
        flightTime.add(36, 0.22); //tuned, 0.22
        flightTime.add(53.6, 0.14); //tuned, 0.14
        flightTime.add(73.5, 0.34); //tuned
        flightTime.add(100, 0.44); //tuned
        flightTime.add(135 , 1.02); //tuned
        flightTime.add(150 , 1.02); //tuned
        flightTime.add(1000, 1.02); //tuned
//        flightTime.add(0, 0); //for turn off
//        flightTime.add(1000, 1.02); //for turn off
        flightTime.createLUT();
    }
    public Pose getBotPose(){
        Alliance alliance = CurrentAlliance.alliance;

        double x = follower.getPose().getX();
        double y = follower.getPose().getY();
        double distance = LocalizationHelper.getTargetDistance(follower.getPose(), alliance);
        double t = flightTime.get(distance) * BotPoseConstants.leadMultiplier;

        double velX = follower.getVelocity().getXComponent() * t;
        double velY = follower.getVelocity().getYComponent() * t;

        double accelX = 0.5 * follower.getAcceleration().getXComponent() * t * t;
        double accelY = 0.5 * follower.getAcceleration().getYComponent() * t * t;

        double newX = x + velX + accelX;
        double newY = y + velY + accelY;

        return new Pose(newX, newY, follower.getHeading());
    }
}
