package org.firstinspires.ftc.teamcode.states.tests;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.states.Management.Handlers.FSM.states.Controllers.subsystems.Hood;
import org.firstinspires.ftc.teamcode.states.Management.Handlers.FSM.states.Controllers.subsystems.Shooter;

public class ShootingTester extends OpMode {
    Hood hood;
    Shooter shooter;
    Follower follower;
    TelemetryManager telemetryManager;
    Pose startPose = new Pose(0, 0, Math.toRadians(90));
    //ShooterController control = new ShooterController(shooter, hood, )
    @Override
    public void init() {
        hood = new Hood(hardwareMap, "hood");
        shooter = new Shooter(hardwareMap, "flyRight", "flyLeft");
        follower = Constants.createFollower(hardwareMap);
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
        follower.setPose(startPose);
        follower.update();
    }

    @Override
    public void loop() {
        double distance = getTargetDistance(follower, alliance);
        hood.angleHood(distance);
        shooter.shoot(distance);
    }
    private final Pose blueGoal = new Pose(0, 138);
    private final Pose redGoal = new Pose(138, 138);
    public static int alliance = 1;
    public double targetDistance;
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
