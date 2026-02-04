package org.firstinspires.ftc.teamcode.RIstates.management.handlers;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RIstates.management.Systems.servo.IntakeController;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.HoodController;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Shooter;

@Configurable
public class ShooterHandler {
    private final TelemetryManager telemetryM;
    private final Follower follower;
    private final Shooter shooter;
    private final HoodController hoodController;
    private final IntakeController intakeController;
//    private final LightController lightController;
    //private final BallController ballController;

    public ShooterHandler(
            TelemetryManager telemetryM,
            Follower follower,
            Shooter shooter,
            HoodController hoodController,
            IntakeController intakeController
//            LightController lightController,
            //BallController ballController
    ) {
        this.telemetryM = telemetryM;
        this.follower = follower;
        this.shooter = shooter;
        this.hoodController = hoodController;
        this.intakeController = intakeController;
//        this.lightController = lightController;
        //this.ballController = ballController;
    }

    public int alliance = 0;
    private static int farDistance = 20;
    private static int farTime = 5000;
    private static int closeTime = 4000;


    public void shoot() {
            double targetDistance = getTargetDistance(follower, alliance);
            //ballController.shootUpdateBallCount();
            shooter.shoot(targetDistance);
            //hoodController.angleHood(targetDistance);
            //intakeController.run();
//            lightController.shooterLightingUpdate(ballController.getBallCount());
    }
    public void constantShoot() {
        shooter.test(500);
    }

    public void whileConstantShootingBelt(Boolean run) {
        if (run) {
            intakeController.run();
        } else {
            intakeController.stop();
        }
    }
    public boolean shooterRunning = false;

    private ElapsedTime timer = new ElapsedTime();
    public boolean shootTimeCheck(double time){
        if (timer.milliseconds() >= time) {
            return true;
        }
        return false;
    }

    public void off(){
        shooter.stop();
//        hoodController.passive();
//        intakeController.stop();
    }



    /*
    Get Target Distance is a method to retrieve target distance
    inputs: robotPose, (turret) mode
    outputs: targetDistance (also printed to panels)
    !! It is never needed to call this method - it is called in shoot !!
     */
    public static double blueX = 0;
    public static double blueY = 138;
    public static double redX = 138;
    public static double redY = 138;
    private Pose blueGoal = new Pose(blueX, blueY);
    private Pose redGoal = new Pose(redX, redY);
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
        if (targetDistance > 0) {
            telemetryM.addData("distance goal", targetDistance);
            return targetDistance;
        } else {
            return 1;
        }
    }
}
