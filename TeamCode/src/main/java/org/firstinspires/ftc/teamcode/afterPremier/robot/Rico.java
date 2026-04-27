package org.firstinspires.ftc.teamcode.afterPremier.robot;

import static com.pedropathing.ivy.groups.Groups.parallel;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.behaviors.ConflictBehavior;
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
    private void setGoalPose(){
        if(a == Alliance.RED){
            goalPose = RobotConstants.redGoal;
        }
        if(a == Alliance.BLUE){
            goalPose = RobotConstants.blueGoal;
        }
    }
    public void periodic(){
        f.update();
        t.aim(goalPose, f.getPose());
        h.angleHood(f.getPose(), goalPose);
        fly.run(f.getPose(), goalPose);
    }
}
