package org.firstinspires.ftc.teamcode.RIstates.management.handlers;

import static org.firstinspires.ftc.teamcode.RIstates.management.Systems.Limelight.LimelightController.found;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Limelight.LimelightController;
import org.firstinspires.ftc.teamcode.utils.timers.SolversTiming;

public class LimelightHandler {
    private final LimelightController llController;
    private final Follower follower;
    SolversTiming timer = new SolversTiming();
    boolean inited = false;
    //makes LimelightHandler and resets timer
    public LimelightHandler(LimelightController control, Follower f){
        llController = control;
        follower = f;
        timer.create();
        timer.setLength(30000);
        timer.resetThenStart();
    }

    public void updatePosition(){
        //initializes llcontroller once timer is up
        if(timer.checkFinished() && !inited){
            llController.init();
            inited = true;
        }
        //if timer is up and ll initialized, looks for atags and relocalizes
        //once relocalized, end ll and reset timer
        if(timer.checkFinished() && inited && !found){
            llController.update();
        }
        else if(found){
            timer.resetThenStart();
            llController.end();
        }
    }

    public boolean checkFound() {
        return found;
    }

}
