package org.firstinspires.ftc.teamcode.states.Management.Handlers.FSM.states.Controllers;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.states.Management.Handlers.FSM.states.Controllers.subsystems.Hood;
import org.firstinspires.ftc.teamcode.states.Management.Handlers.FSM.states.Controllers.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.states.Management.Handlers.FSM.states.Controllers.subsystems.Turret;

@Configurable
public class ShooterController {
    private final Shooter shooter;
    private final Hood hood;
    private final Turret turret;
    private final Follower follower;
    public ShooterController(Shooter shooter, Hood hood, Turret turret, Follower follower){
        this.shooter = shooter;
        this.hood = hood;
        this.follower = follower;
        this.turret = turret;
    }

    ElapsedTime timer = new ElapsedTime();
    public void shoot(int alliance) {
        double targetDistance = getTargetDistance(follower, alliance);
        shooter.shoot(targetDistance);
        hood.angleHood(targetDistance);
        turret.trackGoal(alliance, follower);
    }

    public void shootTimeStart(){
        timer.reset();
    }
    public boolean shootTimeCheck(double time){
        if (timer.milliseconds() >= time) {
            return true;
        }
        return false;
    }

    public void off(){
        shooter.stop();
        hood.passive();
    }

    /*
    Get Target Distance is a method to retrieve target distance
    inputs: robotPose, (turret) mode
    outputs: targetDistance (also printed to panels)
    !! It is never needed to call this method - it is called in shoot !!
     */
    private final Pose blueGoal = new Pose(0, 138);
    private final Pose redGoal = new Pose(138, 138);
    double targetDistance = 0;
    public double getTargetDistance(Follower follower, int alliance){
        if (alliance == 1){
            Pose currentPose = follower.getPose();
            targetDistance = currentPose.distanceFrom(blueGoal);
        }
        else if (alliance == 2){
            Pose currentPose = follower.getPose();
            targetDistance = currentPose.distanceFrom(redGoal);
        }

        return targetDistance;
    }
}
