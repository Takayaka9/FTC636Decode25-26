package org.firstinspires.ftc.teamcode.RIstates.management.handlers;

import static org.firstinspires.ftc.teamcode.RIstates.management.Systems.Limelight.LimelightController.found;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Limelight.LimelightController;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Utilities.timers.SolversTiming;

public class LimelightHandler {
    private final LimelightController llController;
    private final Follower follower;
    SolversTiming timer = new SolversTiming();
    boolean started = false;
    public LimelightHandler(LimelightController control, Follower f){
        llController = control;
        follower = f;
        timer.create();
        timer.setLength(30000);
        timer.resetThenStart();
    }
    public void relocalize(){
        if(timer.checkFinished() && !started){
            llController.init();
            started = true;
        }
        if(timer.checkFinished() && started && !found){
            llController.update();
        }
        else if(found){
            timer.resetThenStart();
            llController.end();
        }
    }

}
