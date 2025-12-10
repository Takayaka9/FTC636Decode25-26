package org.firstinspires.ftc.teamcode.quals.auto;

import static org.firstinspires.ftc.teamcode.quals.QualsTeleOp.Kd;
import static org.firstinspires.ftc.teamcode.quals.QualsTeleOp.Kf;
import static org.firstinspires.ftc.teamcode.quals.QualsTeleOp.Ki;
import static org.firstinspires.ftc.teamcode.quals.QualsTeleOp.Kp;
import static org.firstinspires.ftc.teamcode.quals.QualsTeleOp.beltOn;
import static org.firstinspires.ftc.teamcode.quals.QualsTeleOp.intakeOn;
import static org.firstinspires.ftc.teamcode.quals.QualsTeleOp.offRampPassive;
import static org.firstinspires.ftc.teamcode.quals.QualsTeleOp.offRampPush;
import static org.firstinspires.ftc.teamcode.quals.QualsTeleOp.onRampPassive;
import static org.firstinspires.ftc.teamcode.quals.QualsTeleOp.onRampPush;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandBase.Commands;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.quals.RobotQuals;

@Disabled
@Configurable
@Autonomous(name = "Blue Auto 2")
public class QualsBlueAutoAttempt2 extends LinearOpMode {
    RobotQuals robot;
    private Follower follower;
    private Timer pathTimer, autoTimer, opmodeTimer;
    private int pathState;
    Commands commands;
    CommandScheduler commandScheduler;


    //Poses
    private final Pose startPose = new Pose(59.8, 8.5, Math.toRadians(90));
    private final Pose shootPose = new Pose(59, 103.7, Math.toRadians(141));
    private final Pose prePickup1 = new Pose(38.2, 87.3, Math.toRadians(180));
    private final Pose Pickup1 = new Pose(18.1, 87.3, Math.toRadians(0));
    //back to shoot pose
    private final Pose prePickup2 = new Pose(100, 57, Math.toRadians(0));
    private final Pose Pickup2 = new Pose(120, 57, Math.toRadians(0));
    private final Pose end = new Pose(59.8, 55.5, Math.toRadians(180));
    //back to shoot pose again

    //control point poses
    private final Pose pickupControl1 = new Pose(85, 85, Math.toRadians(0));
    private final Pose shootControl = new Pose(90, 90, Math.toRadians(0));
    private final Pose pickupControl2 = new Pose(75, 57, Math.toRadians(0));
    private Path scorePreload;
    private PathChain Line1, Line2, Line3, Line4, Line5, Line6, Curve7;
    ElapsedTime autoTime = new ElapsedTime();
    ElapsedTime pidTime = new ElapsedTime();
    public double integralSum;
    public double lastError;
    public boolean shooting = false;
    public static int velocity = 4500;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotQuals(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        shooting = false;

        waitForStart();
        follower.followPath(Line1);
        while(follower.isBusy()){
            activateFly();
        }
        shootAllThree();
        while(shooting){
        }
        follower.followPath(Line2);
        while(follower.isBusy()){
            robot.intake.setPower(intakeOn);
            robot.flyRight.setPower(0);
            robot.flyLeft.setPower(0);
        }
        autoTime.reset();
        robot.belt.setPower(beltOn);
        while(autoTime.seconds() < 0.3){
        }
        robot.belt.setPower(0);
        robot.intake.setPower(0);
        robot.onRamp.setPosition(onRampPush);
        autoTime.reset();
        while(autoTime.seconds() < 0.7){
        }
        robot.onRamp.setPosition(onRampPassive);
        robot.intake.setPower(intakeOn);
        robot.belt.setPower(beltOn);
        follower.followPath(Line3);
        while(follower.isBusy()){
        }
        autoTime.reset();
        while(autoTime.seconds() < 0.5){
        }
        robot.intake.setPower(0);
        robot.belt.setPower(-1);
        autoTime.reset();
        while(autoTime.seconds() < 0.2){
        }
        robot.belt.setPower(0);
        follower.followPath(Line4);
        while(follower.isBusy()){
            activateFly();
        }
        shootAllThree();
        while(shooting){
        }
        follower.followPath(Line5);
        while(follower.isBusy()){
        }
    }

    public void buildPaths() {

        Line1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        Line2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, prePickup1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), prePickup1.getHeading())
                .build();

        Line3 = follower.pathBuilder()
                .addPath(new BezierLine(prePickup1, Pickup1))
                .setConstantHeadingInterpolation(Pickup1.getHeading())
                .build();

        Line4 = follower.pathBuilder()
                .addPath(new BezierLine(Pickup1, shootPose))
                .setLinearHeadingInterpolation(Pickup1.getHeading(), shootPose.getHeading())
                .build();

        Line5 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, end))
                .setLinearHeadingInterpolation(shootPose.getHeading(), end.getHeading())
                .build();

        Line6 = follower.pathBuilder()
                .addPath(new BezierLine(prePickup2, Pickup2))
                .setConstantHeadingInterpolation(Pickup2.getHeading())
                .build();

        /*
        Curve4 = follower.pathBuilder()
                .addPath(new BezierCurve(Pickup2, shootControl, shootPose))
                .setLinearHeadingInterpolation(Pickup2.getHeading(), shootPose.getHeading())
                .build();

         */
    }

    public void shootAllThree(){
        shooting = true;
        robot.belt.setPower(1);
        autoTime.reset();
        while(autoTime.seconds() < 0.3){
        }
        robot.belt.setPower(0);
        autoTime.reset();
        robot.offRamp.setPosition(offRampPush);
        while(autoTime.seconds() < 1){
        }
        robot.offRamp.setPosition(offRampPassive);
        robot.belt.setPower(1);
        autoTime.reset();
        while(autoTime.seconds() < 0.3){
        }
        robot.belt.setPower(0);
        autoTime.reset();
        while(autoTime.seconds() < 1){
        }
        robot.belt.setPower(1);
        while(autoTime.seconds() < 0.3){
        }
        robot.belt.setPower(0);
        shooting = false;
    }

    public void activateFly(){
        double error = velocity-(robot.flyRight.getVelocity());
        integralSum += error* pidTime.seconds();
        double derivative = (error- lastError)/ pidTime.seconds();
        lastError = error;

        pidTime.reset();

        double output; // basically the same as the normal PIDControl
        output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (velocity * Kf);
        //telemetryM.addData("output", output);

        robot.flyRight.setPower(Math.max(-1, Math.min(1, output))); //clamping so values do not exceed 1 or -1
        robot.flyLeft.setPower(Math.max(-1, Math.min(1, output)));
    }
}
