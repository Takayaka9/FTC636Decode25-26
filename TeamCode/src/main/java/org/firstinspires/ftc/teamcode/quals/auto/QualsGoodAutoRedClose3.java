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
import static org.firstinspires.ftc.teamcode.quals.QualsTeleOp.shoot2;
import static org.firstinspires.ftc.teamcode.quals.QualsTeleOp.shoot3;
import static org.firstinspires.ftc.teamcode.quals.QualsTeleOp.shoot4;
import static org.firstinspires.ftc.teamcode.quals.QualsTeleOp.shoot5;
import static org.firstinspires.ftc.teamcode.quals.QualsTeleOp.shoot6;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
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
@Autonomous(name = "Quals Red Close Park")
public class QualsGoodAutoRedClose3 extends OpMode {
    RobotQuals robot;
    TelemetryManager telemetryManager;
    private Follower follower;
    //TelemetryManager telemetryManager;
    private final Pose startPose = new Pose(124.723, 121.253, Math.toRadians(36));
    private final Pose shootPose = new Pose(shootX, shootY, Math.toRadians(shootA));
    private final Pose endPose = new Pose(endX, endY, Math.toRadians(endA));
    private final Pose pickUp1 = new Pose(firstPickX, pickupY, Math.toRadians(pickUpA));
    private final Pose pickUp23 = new Pose(pickUpX, pickupY, Math.toRadians(pickUpA));
    public static double pickUpA = -10;
    public static double endX = 123;
    public static double endY = 102;
    public static double endA = 90;
    public static double shootY = 90;
    public static double shootX = 90;
    public static double shootA = 34;
    public static double pickupY = 80;
    public static double firstPickX = 108;
    public static double pickUpX = 10;
    private PathChain InitialShoot, Line2, Line3, Line4, Line5, Park, PickUp1, BackToShoot;
    ElapsedTime autoTime = new ElapsedTime();
    public static double auto1 = 0.3;
    public static double auto2 = 0.67;
    public static double auto3 = 0.1;
    public static double auto4 = 3;
    public static double path1 = 3;
    public static double path2 = 5;
    public static double path3 = 10;
    public static double path4 = 5;
    public static double path5 = 5;
    ElapsedTime pidTime = new ElapsedTime();
    Timer pathTimer;
    private int pathState;
    ElapsedTime shootTime = new ElapsedTime();
    public double integralSum;
    public double lastError;
    public static int velocity = 1650;
    public static boolean firstTime = false;

    @Override
    public void init() {
        robot = new RobotQuals(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        pathTimer = new Timer();
        follower.setStartingPose(startPose);
        firstTime = true;
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
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
        telemetryManager.update();

        telemetryManager.addData("State", autoSteps);
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
                autoTime.reset();
                pathTimer.resetTimer();
                prepFly();
                break;
            case TO_SHOOT1:
                follower.followPath(InitialShoot, true);
                //activateFly();
                if(autoTime.seconds() >= path1){
                    stopMove();
                    activateFly();
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
                    autoSteps = AutoSteps.END;
                }
                break;
            case END:
                follower.followPath(Park);
                if(autoTime.seconds() >= path5){
                    autoSteps = AutoSteps.ENDEND;
                }
                break;
            case ENDEND:
                stopMove();
        }
    }

    public void buildPaths() {

        InitialShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();



        Park = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();




        PickUp1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, pickUp1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickUp1.getHeading())
                .build();


        BackToShoot = follower.pathBuilder()
                //.addPath(new BezierLine(pickUp1, shootPose))
                //.setConstantHeadingInterpolation(pickUp1.getHeading())
                .addPath(new BezierLine(pickUp1, shootPose))
                .setLinearHeadingInterpolation(pickUp1.getHeading(), shootPose.getHeading())
                .build();

    }
}
