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
import static org.firstinspires.ftc.teamcode.quals.QualsTeleOp.shoot2;
import static org.firstinspires.ftc.teamcode.quals.QualsTeleOp.shoot3;
import static org.firstinspires.ftc.teamcode.quals.QualsTeleOp.shoot4;
import static org.firstinspires.ftc.teamcode.quals.QualsTeleOp.shoot5;
import static org.firstinspires.ftc.teamcode.quals.QualsTeleOp.shoot6;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.quals.RobotQuals;

@Configurable
@Autonomous(name = "Red Auto Attempt 2_FSM")
public class QualsRedAutoAttempt2_FSM extends OpMode {
    RobotQuals robot;
    private Follower follower;
    //TelemetryManager telemetryManager;
    private final Pose startPose = new Pose(-59.8, 9.228, Math.toRadians(90));
    private final Pose shootPose = new Pose(shootX, shootY, Math.toRadians(shootA));
    private final Pose prePickup1 = new Pose(firstPickX, pickupY, Math.toRadians(0));
    private final Pose Pickup1 = new Pose(-18.1, pickupY, Math.toRadians(0));
    private final Pose end = new Pose(-59.8, 55.5, Math.toRadians(0));
    public static double shootY = 75;
    public static double shootX = -70;
    public static double shootA = 49;
    public static double pickupY = 87.3;
    public static double firstPickX = -38.2;
    private PathChain Line1, Line2, Line3, Line4, Line5;
    ElapsedTime autoTime = new ElapsedTime();
    public static double auto1 = 0.3;
    public static double auto2 = 0.67;
    public static double auto3 = 0.2;
    public static double path1 = 5;
    public static double path2 = 5;
    public static double path3 = 5;
    public static double path4 = 5;
    public static double path5 = 5;
    ElapsedTime pidTime = new ElapsedTime();
    Timer pathTimer;
    private int pathState;
    ElapsedTime shootTime = new ElapsedTime();
    public double integralSum;
    public double lastError;
    public static int velocity = 4500;
    public static boolean firstTime = false;

    @Override
    public void init() {
        robot = new RobotQuals(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        pathTimer = new Timer();
        follower.setStartingPose(startPose);
        firstTime = true;
    }

    @Override
    public void start() {
        autoSteps = AutoSteps.READY;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        auto();
    }

    public enum AutoSteps{
        READY, TO_SHOOT1, TO_PICKUP1, PICKUP1, PICKUP23, READY_SHOOT, TO_SHOOT2, REV_1, SHOOT_1, REV_2, SHOOT_2, REV_3, SHOOT_3, FINISH, END
    }
    AutoSteps autoSteps = AutoSteps.READY;

    public void prepFly(){
        integralSum = 0;
        //integralSumLeft = 0;
        pidTime.reset();
        lastError = 0;
        //lastErrorLeft = 0;
    }
    public double integralSumLeft;
    public double lastErrorLeft;
    public void activateFly(){
        double error = velocity-(robot.flyRight.getVelocity());
        //double errorLeft = velocity-(robot.flyLeft.getVelocity());
        integralSum += error* pidTime.seconds();
        //integralSumLeft += errorLeft*pidTime.seconds();
        double derivative = (error- lastError)/ pidTime.seconds();
        //double derivativeLeft = (errorLeft - lastErrorLeft) / pidTime.seconds();
        lastError = error;
        //lastErrorLeft = error;

        pidTime.reset();

        double output; // basically the same as the normal PIDControl
        output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (velocity * Kf);
        //telemetryM.addData("output", output);

        //double outputLeft;
        //outputLeft = (errorLeft * Kp) + (derivativeLeft * Kd) + (integralSumLeft * Ki) + (velocity * Kf);

        robot.flyRight.setPower(Math.max(-1, Math.min(1, output))); //clamping so values do not exceed 1 or -1
        robot.flyLeft.setPower(Math.max(-1, Math.min(1, output)));
        //robot.flyLeft.setPower(Math.max(-1, Math.min(1, outputLeft)));
    }

    public void stopMove(){
        robot.rightBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.leftFront.setPower(0);
    }

    @Override
    public void stop() {
        super.stop();
    }

    public void auto(){
        switch(autoSteps){
            case READY:
                robot.onRamp.setPosition(onRampPassive);
                robot.offRamp.setPosition(offRampPassive);
                robot.belt.setPower(0);
                robot.flyRight.setPower(0);
                robot.flyLeft.setPower(0);
                robot.intake.setPower(0);
                autoSteps = AutoSteps.TO_SHOOT1;
                prepFly();
                break;
            case TO_SHOOT1:
                follower.followPath(Line1);
                autoTime.reset();
                pathTimer.resetTimer();
                activateFly();
                if(autoTime.seconds() >= path1){
                    stopMove();
                    activateFly();
                    autoSteps = AutoSteps.REV_1;
                }
                break;
            case REV_1:
                robot.belt.setPower(0);
                robot.intake.setPower(0);
                robot.offRamp.setPosition(offRampPassive);
                robot.onRamp.setPosition(onRampPassive);
                shootTime.reset();
                autoSteps = AutoSteps.SHOOT_1;
                break;
            case SHOOT_1:
                robot.belt.setPower(1);
                shootTime.reset();
                autoSteps = AutoSteps.REV_2;
                break;
            case REV_2:
                if(shootTime.seconds() >= shoot2){
                    robot.belt.setPower(0);
                    robot.offRamp.setPosition(offRampPush);
                    shootTime.reset();
                    autoSteps = AutoSteps.SHOOT_2;
                }
                break;
            case SHOOT_2:
                if(shootTime.seconds() >= shoot3){
                    robot.offRamp.setPosition(offRampPassive);
                    robot.belt.setPower(1);
                    shootTime.reset();
                    autoSteps = AutoSteps.REV_3;
                }
                break;
            case REV_3:
                if (shootTime.seconds() >= shoot4) {
                    robot.belt.setPower(0);
                    shootTime.reset();
                    autoSteps = AutoSteps.SHOOT_3;
                }
                break;
            case SHOOT_3:
                if(shootTime.seconds() >= shoot5){
                    robot.belt.setPower(1);
                    shootTime.reset();
                    autoSteps = AutoSteps.FINISH;
                }
                break;
            case FINISH:
                if(shootTime.seconds()>= shoot6 && firstTime){
                    robot.belt.setPower(0);
                    firstTime = false;
                    autoSteps = AutoSteps.TO_PICKUP1;
                }
                if(shootTime.seconds()>= shoot6 && !firstTime){
                    robot.belt.setPower(0);
                    autoSteps = AutoSteps.END;
                }
                break;
            case TO_PICKUP1:
                follower.followPath(Line2);
                autoTime.reset();
                pathTimer.resetTimer();
                robot.intake.setPower(intakeOn);
                robot.belt.setPower(beltOn);
                robot.flyRight.setPower(0);
                robot.flyLeft.setPower(0);
                if(autoTime.seconds() >= path2){
                    autoTime.reset();
                    stopMove();
                    autoSteps = AutoSteps.PICKUP1;
                }
                break;
            case PICKUP1:
                robot.intake.setPower(0);
                if(autoTime.seconds() >= auto1){
                    robot.belt.setPower(0);
                    robot.onRamp.setPosition(onRampPush);
                    autoTime.reset();
                    autoSteps = AutoSteps.PICKUP23;
                }
                break;
            case PICKUP23:
                follower.followPath(Line3);
                autoTime.reset();
                pathTimer.resetTimer();
                robot.intake.setPower(intakeOn);
                robot.belt.setPower(beltOn);
                robot.onRamp.setPosition(onRampPassive);
                if(autoTime.seconds() >= path3){
                    stopMove();
                    robot.intake.setPower(0);
                    robot.belt.setPower(-1);
                    autoTime.reset();
                    prepFly();
                    autoSteps = AutoSteps.READY_SHOOT;
                }
                break;
            case READY_SHOOT:
                follower.followPath(Line4);
                pathTimer.resetTimer();
                if(autoTime.seconds() >= auto3){
                    robot.belt.setPower(0);
                    activateFly();
                }
                if(autoTime.seconds() >= path4){
                    stopMove();
                    autoSteps = AutoSteps.REV_1;
                }
                break;
            case END:
                follower.followPath(Line5);
                if(autoTime.seconds() >= path5){
                    stopMove();
                }
                break;
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
        /*
        Line6 = follower.pathBuilder()
                .addPath(new BezierLine(prePickup2, Pickup2))
                .setConstantHeadingInterpolation(Pickup2.getHeading())
                .build();

         */

        /*
        Curve4 = follower.pathBuilder()
                .addPath(new BezierCurve(Pickup2, shootControl, shootPose))
                .setLinearHeadingInterpolation(Pickup2.getHeading(), shootPose.getHeading())
                .build();

         */
    }
}
