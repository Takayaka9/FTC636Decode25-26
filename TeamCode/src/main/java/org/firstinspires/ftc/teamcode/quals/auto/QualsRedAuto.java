package org.firstinspires.ftc.teamcode.quals.auto;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.sun.tools.javac.comp.Todo;

import org.firstinspires.ftc.teamcode.CommandBase.Commands;
import org.firstinspires.ftc.teamcode.CommandBase.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.quals.QualsTeleOp;
import org.firstinspires.ftc.teamcode.quals.RobotQuals;

@Configurable
@Autonomous(name = "Quals Auto Red")
public class QualsRedAuto extends LinearOpMode{
    RobotQuals robot;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    Commands commands;
    CommandScheduler commandScheduler;


    //Poses
    private final Pose startPose = new Pose(117, 129, Math.toRadians(37));
    private final Pose shootPose = new Pose(75, 75, Math.toRadians(45));
    private final Pose prePickup1 = new Pose(100, 80, Math.toRadians(0));
    private final Pose Pickup1 = new Pose(120, 80, Math.toRadians(0));
    //back to shoot pose
    private final Pose prePickup2 = new Pose(100, 57, Math.toRadians(0));
    private final Pose Pickup2 = new Pose(120, 57, Math.toRadians(0));
    //back to shoot pose again

    //control point poses
    private final Pose pickupControl1 = new Pose(85, 85, Math.toRadians(0));
    private final Pose shootControl = new Pose(90, 90, Math.toRadians(0));
    private final Pose pickupControl2 = new Pose(75, 57, Math.toRadians(0));


    private Path scorePreload;
    private PathChain Line1, Curve2, Line3, Curve4, Curve5, Line6, Curve7;

    @Override
    public void runOpMode() throws InterruptedException {

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        robot = new RobotQuals(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        waitForStart();

        opmodeTimer.resetTimer();
        setPathState(0);

        runPath(Line1);
        sleep(500);
        robot.shooterPIDF(QualsTeleOp.velocity);
        sleep(4000);
        robot.belt.setPower(QualsTeleOp.beltOn);
        sleep(5000);
        robot.shooterPIDF(0);
        robot.belt.setPower(0);
        runPath(Curve2);
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    private void runPath(PathChain p) {
        follower.followPath(p);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
    }



    // USE COMMAND SYSTEM IN THE NEXT BIT I HAVEN'T DONE IT YET
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(Line1);
                setPathState(1);
                break;
                /*
            case 1:
                follower.followPath(Line2);
                setPathState(1);
                break;
            case 2:
                follower.followPath(Curve3);
                setPathState(1);
                break;
            case 3:

                 */

        }
    }

    public void buildPaths() {

        Line1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        Curve2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, pickupControl1, prePickup1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), prePickup1.getHeading())
                .build();

        Line3 = follower.pathBuilder()
                .addPath(new BezierLine(prePickup1, Pickup1))
                .setConstantHeadingInterpolation(Pickup1.getHeading())
                .build();

        Curve4 = follower.pathBuilder()
                .addPath(new BezierCurve(Pickup1, shootControl, shootPose))
                .setLinearHeadingInterpolation(Pickup1.getHeading(), shootPose.getHeading())
                .build();

        Curve5 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, pickupControl2,  prePickup2))
                .setLinearHeadingInterpolation(shootPose.getHeading(), prePickup2.getHeading())
                .build();

        Line6 = follower.pathBuilder()
                .addPath(new BezierLine(prePickup2, Pickup2))
                .setConstantHeadingInterpolation(Pickup2.getHeading())
                .build();

        Curve4 = follower.pathBuilder()
                .addPath(new BezierCurve(Pickup2, shootControl, shootPose))
                .setLinearHeadingInterpolation(Pickup2.getHeading(), shootPose.getHeading())
                .build();
    }

    SequentialCommandGroup auto = new SequentialCommandGroup(
            new InstantCommand(() -> {
                robot.belt.setPower(0.75); //example
            })
    );
}