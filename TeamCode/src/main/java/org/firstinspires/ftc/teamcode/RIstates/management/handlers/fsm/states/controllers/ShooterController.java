package org.firstinspires.ftc.teamcode.RIstates.management.handlers.fsm.states.controllers;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RIstates.management.handlers.fsm.states.controllers.subsystems.Hood;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.fsm.states.controllers.subsystems.Shooter;
//import org.firstinspires.ftc.teamcode.RIstates.management.handlers.fsm.states.controllers.subsystems.Turret;

@Configurable
public class ShooterController {
    private final TelemetryManager telemetryM;
    private final Follower follower;
    private final Shooter shooter;
    private final Hood hood;
    //private final Turret turret;
    private final BeltController beltController;
    public ShooterController(TelemetryManager telemetryM, Follower follower, Shooter shooter, Hood hood, BeltController beltController){
        this.telemetryM = telemetryM;
        this.follower = follower;
        this.shooter = shooter;
        this.hood = hood;
        //this.turret = turret;
        this.beltController = beltController;
    }

    public int alliance = 0;
    private static int farDistance = 20;
    private static int farTime = 5000;
    private static int closeTime = 4000;


    public void shoot() {
//        if (!beltController.checkShotCounter()) {
//            timer.reset();
//            double targetDistance = getTargetDistance(follower, alliance);
//            shooter.shoot(targetDistance);
//            hood.angleHood(targetDistance);
//            //turret.trackGoal(alliance, follower);
//            beltController.run();
//            shooterRunning = true;
//        }
//        if (beltController.checkShotCounter()) {
//            shooterRunning = false;
//            off();
//            return true;
//        }
//        return false;
            double targetDistance = getTargetDistance(follower, alliance);
            shooter.shoot(targetDistance);
            hood.angleHood(targetDistance);
            //turret.trackGoal(alliance, follower);
            beltController.run();
    }
    public boolean shooterRunning = false;
    public void enableHardware() {


    }

    private ElapsedTime timer = new ElapsedTime();
    public boolean shootTimeCheck(double time){
        if (timer.milliseconds() >= time) {
            return true;
        }
        return false;
    }

    public void off(){
        shooter.stop();
        hood.passive();
        beltController.belt.stop();
    }

    /*
    Get Target Distance is a method to retrieve target distance
    inputs: robotPose, (turret) mode
    outputs: targetDistance (also printed to panels)
    !! It is never needed to call this method - it is called in shoot !!
     */
    private final Pose blueGoal = new Pose(0, 138);
    private final Pose redGoal = new Pose(138, 138);
    private static double targetDistance = 0;
    public double getTargetDistance(Follower follower, int alliance){
        if (alliance == 1){
            Pose currentPose = follower.getPose();
            targetDistance = currentPose.distanceFrom(blueGoal);
        }
        else if (alliance == 2){
            Pose currentPose = follower.getPose();
            targetDistance = currentPose.distanceFrom(redGoal);
        }
        if (targetDistance > 20) {
            return targetDistance;
        } else {
            return 1;
        }
    }
}
