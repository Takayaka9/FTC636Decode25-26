package org.firstinspires.ftc.teamcode.states.subsystems;

import static org.firstinspires.ftc.teamcode.states.StatesTeleOp.TurretModes.BLUE;
import static org.firstinspires.ftc.teamcode.states.StatesTeleOp.TurretModes.RED;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.states.StatesTeleOp;

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

    public void shoot(int alliance){
        double targetDistance = getTargetDistance(follower, alliance);

        shooter.shoot(targetDistance);
        hood.angleHood(targetDistance);
        turret.trackGoal(alliance, follower);
    }

    public void off(){
        shooter.off();
        hood.passive();
    }

    /*
    Get Target Distance is a method to retrieve target distance
    inputs: robotPose, (turret) mode
    outputs: targetDistance (also printed to panels)
    call once in opmode before using methods requiring 'targetDistance' then pass into method
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
