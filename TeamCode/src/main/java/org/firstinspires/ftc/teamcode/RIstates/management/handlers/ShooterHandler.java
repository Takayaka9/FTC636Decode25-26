package org.firstinspires.ftc.teamcode.RIstates.management.handlers;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RIstates.management.Systems.servo.IntakeController;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Shooter;
import org.firstinspires.ftc.teamcode.utils.alliance.CurrentAlliance;
import org.firstinspires.ftc.teamcode.utils.alliance.GetTargetDistance;

@Configurable
public class ShooterHandler {
    private final TelemetryManager telemetryM;
    private final Follower follower;
    private final Shooter shooter;
    private final IntakeController intakeController;
    private final GetTargetDistance getTargetDistance;

    public ShooterHandler(
            TelemetryManager telemetryM,
            Follower follower,
            Shooter shooter,
            IntakeController intakeController,
            CurrentAlliance alliance
    ) {
        this.telemetryM = telemetryM;
        this.follower = follower;
        this.shooter = shooter;
        this.intakeController = intakeController;
        this.getTargetDistance = new GetTargetDistance();
    }

    public int alliance = 0;


    public void shoot() {
        double targetDistance = getTargetDistance.getTargetDistance(follower.getPose(), CurrentAlliance.alliance);
        shooter.shoot(targetDistance);
    }

    public static double allowedError = 50;

    public boolean inRPMRange() {
        return Math.abs(shooter.getError()) < allowedError;
    }

    public void autoShoot() {
        if (inRPMRange()) {
            intakeController.run();
        } else {
            intakeController.stop();
        }
    }

    public boolean shooterRunning = false;


}