package org.firstinspires.ftc.teamcode.NewEnglands.utils.alliance;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

public abstract class TDistHelper {
    /*
    Get Target Distance is a method to retrieve target distance
    inputs: robotPose, (turret) mode
    outputs: targetDistance (also printed to panels)
    !! It is never needed to call this method - it is called in shoot !!
     */
    @Configurable
    public static class goalPoses {
        public static double blueX = 4;
        public static double blueY = 144;
        public static double redX = 144;
        public static double redY = 140;
    }

    public static double getTargetDistance(Pose currentPose, Alliance alliance){
        double targetDistance = 0;
        if (alliance == Alliance.RED){
            Pose redGoal = new Pose(goalPoses.redX, goalPoses.redY);
            targetDistance = currentPose.distanceFrom(redGoal);
        }
        else if (alliance == Alliance.BLUE){
            Pose blueGoal = new Pose(goalPoses.blueX, goalPoses.blueY);
            targetDistance = currentPose.distanceFrom(blueGoal);
        }
        if (targetDistance > 0) {
            return targetDistance;
        } else {
            return 1;
        }
    }
}