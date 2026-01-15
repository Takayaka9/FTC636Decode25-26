package org.firstinspires.ftc.teamcode.RIstates.management.handlers;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Limelight.LimelightController;

public class LocalizationHandler {
    private final LimelightController llController;
    private final Follower follower;
    public LocalizationHandler(LimelightController control, Follower f){
        llController = control;
        follower = f;
    }



}
