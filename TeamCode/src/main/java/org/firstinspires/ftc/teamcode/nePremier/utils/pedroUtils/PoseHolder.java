package org.firstinspires.ftc.teamcode.nePremier.utils.pedroUtils;

import com.pedropathing.geometry.Pose;

public class PoseHolder {
    private static Pose pose = null;

    public static Pose getPose() {
        if (pose != null) return pose;
        return new Pose(72,72,0);
    }

    public static void setPose(Pose newPose) {
        pose = newPose;
    }
}
