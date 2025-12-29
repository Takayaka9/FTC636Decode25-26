package org.firstinspires.ftc.teamcode.RIstates.management.handlers.fsm.states.controllers;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RIstates.management.handlers.fsm.states.controllers.subsystems.Hood;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.fsm.states.controllers.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.fsm.states.controllers.subsystems.Turret;

@Configurable
public class ShooterController {
    private final TelemetryManager telemetryM;
    private final Follower follower;
    private final Shooter shooter;
    private final Hood hood;
    private final Turret turret;

    public ShooterController(TelemetryManager telemetryM, Follower follower, Shooter shooter, Hood hood, Turret turret){
        this.telemetryM = telemetryM;
        this.follower = follower;
        this.shooter = shooter;
        this.hood = hood;
        this.turret = turret;
    }

    public int alliance = 0;
    private static int farDistance = 20;
    private static int farTime = 5000;
    private static int closeTime = 4000;


    public void shoot() {
        shootTimeStart();
        if (!shooterRunning){
            enableHardware(false);
        }
        if (shooterRunning){
            enableHardware(true);
        }
        if (targetDistance < 20) {
            if (shootTimeCheck(farTime)) {
                shooterRunning = false;
            }
        }
        else {
            if (shootTimeCheck(closeTime)) {
                shooterRunning = false;
            }
        }
        telemetryM.addData("shooter running", shooterRunning);
    }
    public boolean shooterRunning;
    public void enableHardware(boolean checking) {
        if (!checking) {
            double targetDistance = getTargetDistance(follower, alliance);
            shooter.shoot(targetDistance);
            hood.angleHood(targetDistance);
            turret.trackGoal(alliance, follower);
            shooterRunning = true;
        }
    }

    private ElapsedTime timer = new ElapsedTime();
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
    private double targetDistance = 0;
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
