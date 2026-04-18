package org.firstinspires.ftc.teamcode.nePremier.robot.systems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.LocalizationHelper;
import org.firstinspires.ftc.teamcode.nePremier.utils.commandUtils.BaseSubsystem;

@Configurable
public class BotPose extends BaseSubsystem {
    Follower follower;
    InterpLUT flightTime = new InterpLUT();

    @SuppressWarnings("FieldMayBeFinal")
    @Configurable
    public static class BotPoseConstants {
        public static double leadMultiplier = 1; //1.6 on old code
        public static int predictionIterations = 3;
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
        flightTime.add(100000000, 1.02); //tuned
//        flightTime.add(0, 0); //for turn off
//        flightTime.add(1000, 1.02); //for turn off
        flightTime.createLUT();
    }
    public Pose getBotPose(){
        Pose currentPose = follower.getPose();
        double x = currentPose.getX();
        double y = currentPose.getY();
        double velX = follower.getVelocity().getXComponent();
        double velY = follower.getVelocity().getYComponent();
        double accelX = follower.getAcceleration().getXComponent();
        double accelY = follower.getAcceleration().getYComponent();

        Pose predictedPose = currentPose;
        int iterations = Math.max(1, BotPoseConstants.predictionIterations);

        for (int i = 0; i < iterations; i++) {
            double distance = LocalizationHelper.getTargetDistance(predictedPose);
            double t = flightTime.get(distance) * BotPoseConstants.leadMultiplier;

            double newX = x + (velX * t) + (0.5 * accelX * t * t);
            double newY = y + (velY * t) + (0.5 * accelY * t * t);

            predictedPose = new Pose(newX, newY, follower.getHeading());
        }

        return predictedPose;
    }
}
