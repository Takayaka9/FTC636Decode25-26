package org.firstinspires.ftc.teamcode.zquals.auto;

import static org.firstinspires.ftc.teamcode.zquals.QualsTeleOp.Kd;
import static org.firstinspires.ftc.teamcode.zquals.QualsTeleOp.Kf;
import static org.firstinspires.ftc.teamcode.zquals.QualsTeleOp.Ki;
import static org.firstinspires.ftc.teamcode.zquals.QualsTeleOp.Kp;
import static org.firstinspires.ftc.teamcode.zquals.QualsTeleOp.offRampPassive;
import static org.firstinspires.ftc.teamcode.zquals.QualsTeleOp.offRampPush;
import static org.firstinspires.ftc.teamcode.zquals.QualsTeleOp.onRampPassive;
import static org.firstinspires.ftc.teamcode.zquals.QualsTeleOp.shoot2;
import static org.firstinspires.ftc.teamcode.zquals.QualsTeleOp.shoot3;
import static org.firstinspires.ftc.teamcode.zquals.QualsTeleOp.shoot4;
import static org.firstinspires.ftc.teamcode.zquals.QualsTeleOp.shoot5;
import static org.firstinspires.ftc.teamcode.zquals.QualsTeleOp.shoot6;
import static org.firstinspires.ftc.teamcode.zquals.auto.QualsGoodAutoBlueClose2.auto4;
import static org.firstinspires.ftc.teamcode.zquals.auto.QualsGoodAutoBlueClose2.path5;

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
import org.firstinspires.ftc.teamcode.zquals.RobotQuals;

@Configurable
@Autonomous(name = "Quals Blue Far")
public class QualsBlueAutoAttempt2_FSM extends OpMode {
    RobotQuals robot;
    private Follower follower;
    //TelemetryManager telemetryManager;
    private final Pose startPose = new Pose(59.8, 9.228, Math.toRadians(90));
    private final Pose shootPose = new Pose(shootX, shootY, Math.toRadians(shootA));
    private final Pose prePickup1 = new Pose(firstPickX, pickupY, Math.toRadians(180));
    private final Pose Pickup1 = new Pose(18.1, pickupY, Math.toRadians(180));
    private final Pose end = new Pose(endX, endY, Math.toRadians(endA));
    public static double endX = 59.8;
    public static double endY = 55.5;
    public static double endA = 180;
    private PathChain Line1, Line2, Line3, Line4, Line5;
    public static double shootX = 59.8;
    public static double shootA = 113;
    public static double shootY = 20;
    public static double pickupY = 87.3;
    public static double firstPickX = 38.2;
    ElapsedTime autoTime = new ElapsedTime();
    public static double auto1 = 0.3;
    public static double auto2 = 0.67;
    public static double auto3 = 0.2;
    public static double path1 = 2;
    public static boolean autoShoot;
    ElapsedTime pidTime = new ElapsedTime();
    Timer pathTimer;
    private int pathState;
    ElapsedTime shootTime = new ElapsedTime();
    public double integralSum;
    public double lastError;
    public static int velocity = 2100;
    public static boolean firstTime = false;

    @Override
    public void init() {
        robot = new RobotQuals(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        pathTimer = new Timer();
        follower.setStartingPose(startPose);
        firstTime = true;
        autoShoot = false;
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
        if(autoShoot){
            activateFly();
        }
        if(!autoShoot){
            robot.flyLeft.setPower(0);
            robot.flyRight.setPower(0);
        }
    }

    public enum AutoSteps{
        READY, TO_SHOOT1, TO_PICKUP1, PICKUP1, PICKUP23, READY_SHOOT, TO_SHOOT2, REV_1, SHOOT_1, REV_2, SHOOT_2, REV_3, SHOOT_3, FINISH, END, ENDEND
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
    double targetTicks = velocity * 28.0 / 60.0;
    public void activateFly(){
        double error = targetTicks-(robot.flyRight.getVelocity());
        double dt = pidTime.seconds();
        if (dt < 0.0001) dt = 0.0001;
        integralSum += error* dt;
        double derivative = (error- lastError)/ dt;
        lastError = error;

        pidTime.reset();

        double output; // basically the same as the normal PIDControl
        output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (targetTicks * Kf);

        robot.flyRight.setPower(output); //clamping so values do not exceed 1 or -1
        robot.flyLeft.setPower(output);
        //robot.flyLeft.setPower(Math.max(-1, Math.min(1, outputLeft)));
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
                autoTime.reset();
                pathTimer.resetTimer();
                follower.followPath(Line1);
                prepFly();
                break;
            case TO_SHOOT1:
                //follower.followPath(Line1);
                if(!follower.isBusy()){
                    //stopMove();
                    //activateFly();
                    autoShoot = true;
                    pathTimer.resetTimer();
                    autoSteps = AutoSteps.REV_1;
                }
                break;
            case REV_1:
                robot.belt.setPower(0);
                robot.intake.setPower(0);
                robot.offRamp.setPosition(offRampPassive);
                robot.onRamp.setPosition(onRampPassive);
                shootTime.reset();
                autoTime.reset();
                autoSteps = AutoSteps.SHOOT_1;
                break;
            case SHOOT_1:
                if(autoTime.seconds() >= auto4){
                    robot.belt.setPower(1);
                    shootTime.reset();
                    autoSteps = AutoSteps.REV_2;
                }
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
                if(shootTime.seconds() > shoot6){
                    robot.belt.setPower(0);
                    autoTime.reset();
                    pathTimer.resetTimer();
                    follower.followPath(Line5);
                    autoSteps = AutoSteps.END;
                }
                break;
            case END:
                //follower.followPath(Line5);
                if(autoTime.seconds() >= path5){
                    autoSteps = AutoSteps.ENDEND;
                }
                break;
            case ENDEND:
                //stopMove();
                autoShoot = false;
        }
    }

    public void stopMove(){
        robot.rightBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.flyRight.setPower(0);
        robot.flyLeft.setPower(0);
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
