package org.firstinspires.ftc.teamcode.afterPremier.robot;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.afterPremier.robot.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.afterPremier.robot.subsystems.Hood;
import org.firstinspires.ftc.teamcode.afterPremier.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.afterPremier.robot.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.afterPremier.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.afterPremier.util.Alliance;
import org.firstinspires.ftc.teamcode.afterPremier.util.RobotConstants;
import org.firstinspires.ftc.teamcode.nePremier.pedro.autoConstants.AutoConstants;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.CurrentAlliance;
import org.firstinspires.ftc.teamcode.nePremier.utils.pedroUtils.BasePathUpdate;
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
    public Rico(HardwareMap hardwareMap, Alliance alliance){
        a = alliance;
        f = Constants.createFollower(hardwareMap);
        t = new Turret(hardwareMap);
        h = new Hood(hardwareMap);
        i = new Intake(hardwareMap);
        fly = new Flywheel(hardwareMap);
        s = new Stopper(hardwareMap);
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
        t.aim(goalPose, f.getPose());
        h.angleHood(f.getPose(), goalPose);
        fly.setTarget(f.getPose(), goalPose);
        fly.run(fly.getTarget());
    }
    //run in loop for auto
    public void autoLoop(){
        RobotConstants.turretPosTransfer = t.getPosition();
        RobotConstants.setPose(f.getPose());
    }
    public CommandBuilder shoot(){
        return sequential(
                i.off(),
                waitUntil(() -> fly.targetReached(fly.getTarget())),
                s.open(),
                waitMs(100),
                i.in()
        );
    }
    public CommandBuilder autoShoot(){
        return sequential(
                shoot(),
                waitMs(1500)
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
                .setTValueConstraint(AutoConstants.fleePathTValue)
                .build();
        return fleePath;
    }
}
