package org.firstinspires.ftc.teamcode.RIstates.management.pedro;

import com.pedropathing.geometry.Pose;

public class PoseStorage {
    public static Pose endPose;

    public void updatePose(Pose currentPose) {
        if (currentPose != null) {
            endPose = currentPose;
        }
    }

    public Pose getPose(){
        if (endPose != null) {
            return endPose;
        }
        else {
            return new Pose(0, 0, 0);
        }
    }

}
