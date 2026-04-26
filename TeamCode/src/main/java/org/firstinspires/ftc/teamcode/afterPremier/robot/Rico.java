package org.firstinspires.ftc.teamcode.afterPremier.robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.afterPremier.robot.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.afterPremier.robot.subsystems.Hood;
import org.firstinspires.ftc.teamcode.afterPremier.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.afterPremier.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.afterPremier.util.Alliance;
import org.firstinspires.ftc.teamcode.afterPremier.util.RobotConstants;

public class Rico {
    public final Turret t;
    public final Hood h;
    public final Intake i;
    public final Flywheel fly;
    public Pose goalPose = RobotConstants.redGoal;
    Alliance a;
    public Rico(HardwareMap hardwareMap, Alliance alliance){
        a = alliance;
        t = new Turret(hardwareMap);
        h = new Hood(hardwareMap);
        i = new Intake(hardwareMap);
        fly = new Flywheel();
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
}
