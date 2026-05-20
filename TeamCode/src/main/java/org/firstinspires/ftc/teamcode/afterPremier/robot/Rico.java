package org.firstinspires.ftc.teamcode.afterPremier.robot;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.afterPremier.robot.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.afterPremier.robot.subsystems.Hood;
import org.firstinspires.ftc.teamcode.afterPremier.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.afterPremier.robot.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.afterPremier.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.afterPremier.util.Alliance;
import org.firstinspires.ftc.teamcode.afterPremier.util.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Rico {
    public final Turret t;
    public final Hood h;
    public final Intake i;
    public final Flywheel fly;
    public final Stopper s;
    public Pose goalPose = RobotConstants.redGoal;
    public Alliance a;
    public final Follower f;
    public final TelemetryManager telemetry;
    public Rico(HardwareMap hardwareMap, Alliance alliance){
        a = alliance;
        f = Constants.createFollower(hardwareMap);
        t = new Turret(hardwareMap);
        h = new Hood(hardwareMap);
        i = new Intake(hardwareMap);
        fly = new Flywheel(hardwareMap);
        s = new Stopper(hardwareMap);
        telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        setGoalPose();
    }
    //set goal pose for red/blue
    private void setGoalPose(){
        if(a == Alliance.RED){
            goalPose = RobotConstants.redGoal;
        }
        if(a == Alliance.BLUE){
            goalPose = RobotConstants.blueGoal;
        }
    }
    //run constantly
    public void periodic(){
        f.update();
        telemetry.update();
        Pose goal = moveGoal(goalPose); //change all parameters under to goalPose for no sotm
        t.aim(goal, f.getPose(), f);
        h.angleHood(f.getPose(), goal);
        fly.setTarget(f.getPose(), goal);
        fly.run(fly.getTarget());
    }
    private Pose moveGoal(Pose goal){
        double flightTime = t.getLUT(goal.distanceFrom(f.getPose()));

        double velX = f.getVelocity().getXComponent();
        double velY = f.getVelocity().getYComponent();

        double accelX = f.getAcceleration().getXComponent();
        double accelY = f.getAcceleration().getYComponent();

        double flightTimeSquared = flightTime * flightTime;
        double newX = goal.getX() - (velX * flightTime);// + accelX * flightTimeSquared / 2);
        double newY = goal.getY() - (velY * flightTime);// + accelY * flightTimeSquared / 2);

        return new Pose(newX, newY);
    }
    //run in loop for auto
    public void autoLoop(){
        RobotConstants.turretPosTransfer = t.getPosition();
        RobotConstants.setPose(f);
    }
    public CommandBuilder shoot(){
        return sequential(
                i.off(),
                waitUntil(() -> fly.targetReached(fly.getTarget())),
                s.open(),
                waitMs(50),
                i.in(),
                waitMs(1400)
        ).setPriority(1);
    }
    public CommandBuilder autoShoot(){
        return sequential(
                shoot(),
                s.close()
        );
    }
    public PathChain createFleePath() {
        Pose fleePose = null;
        switch(a) {
            case RED:
                fleePose = new Pose(96, 58);
                break;
            case BLUE:
                fleePose = new Pose(48, 58);
                break;
        }
        PathChain fleePath;
        fleePath = f.pathBuilder()
                .addPath(new BezierLine(f.getPose(), fleePose))
                .setTangentHeadingInterpolation()
                .setBrakingStrength(0)
                .setTValueConstraint(0.6)
                .build();
        return fleePath;
    }
}
