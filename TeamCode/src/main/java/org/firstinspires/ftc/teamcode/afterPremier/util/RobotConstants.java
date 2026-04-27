package org.firstinspires.ftc.teamcode.afterPremier.util;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class RobotConstants {
    public static Pose blueGoal= new Pose(0, 140);
    public static Pose redGoal = new Pose(144, 140);

    //utils
    public static Pose lastPose = null;
    public static Pose getPose(){
        if(lastPose == null){
            return new Pose(72, 72, 0);
        }
        else{
            return lastPose;
        }
    }
    public static void setPose(Pose p){
        lastPose = p;
    }
    public static double turretPosTransfer = 0;
}
