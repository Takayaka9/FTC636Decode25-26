package org.firstinspires.ftc.teamcode.quals.auto;

import static org.firstinspires.ftc.teamcode.quals.QualsTeleOp.Kd;
import static org.firstinspires.ftc.teamcode.quals.QualsTeleOp.Kf;
import static org.firstinspires.ftc.teamcode.quals.QualsTeleOp.Ki;
import static org.firstinspires.ftc.teamcode.quals.QualsTeleOp.Kp;
import static org.firstinspires.ftc.teamcode.quals.QualsTeleOp.offRampPassive;
import static org.firstinspires.ftc.teamcode.quals.QualsTeleOp.offRampPush;
import static org.firstinspires.ftc.teamcode.quals.QualsTeleOp.onRampPassive;
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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.quals.QualsTeleOp;
import org.firstinspires.ftc.teamcode.quals.RobotQuals;

@Configurable
@Autonomous(name = "Blue Auto Attempt 2_FSM")
public class QualsBlueAutoAttempt2_FSM extends OpMode {
    RobotQuals robot;
    private Follower follower;
    private final Pose startPose = new Pose(59.8, 8.5, Math.toRadians(90));
    private final Pose shootPose = new Pose(59, 103.7, Math.toRadians(141));
    private final Pose prePickup1 = new Pose(38.2, 87.3, Math.toRadians(180));
    private final Pose Pickup1 = new Pose(18.1, 87.3, Math.toRadians(0));
    private final Pose end = new Pose(59.8, 55.5, Math.toRadians(180));
    private PathChain Line1, Line2, Line3, Line4, Line5;
    ElapsedTime autoTime = new ElapsedTime();
    ElapsedTime pidTime = new ElapsedTime();
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
        follower.setStartingPose(startPose);
        firstTime = true;
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        switch(autoSteps){
            case READY:
                robot.onRamp.setPosition(onRampPassive);
                robot.offRamp.setPosition(offRampPassive);
                robot.belt.setPower(0);
                robot.flyRight.setPower(0);
                robot.flyLeft.setPower(0);
                robot.intake.setPower(0);
                autoSteps = AutoSteps.TO_SHOOT1;
                break;
            case TO_SHOOT1:
                follower.followPath(Line1);
                activateFly();
                if(!follower.isBusy()){
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
                    autoSteps = AutoSteps.TO_SHOOT2;
                }
                if(shootTime.seconds()>= shoot6 && !firstTime){
                    robot.belt.setPower(0);
                    firstTime = false;
                    //autoSteps = AutoSteps.somethingelse;
                }
                break;
        }
    }

    public enum AutoSteps{
        READY, TO_SHOOT1, TO_PICKUP, TO_SHOOT2, REV_1, SHOOT_1, REV_2, SHOOT_2, REV_3, SHOOT_3, FINISH
    }
    AutoSteps autoSteps = AutoSteps.READY;

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

    @Override
    public void stop() {
        super.stop();
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
