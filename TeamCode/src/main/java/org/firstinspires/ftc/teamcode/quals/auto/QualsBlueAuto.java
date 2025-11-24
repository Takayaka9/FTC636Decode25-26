package org.firstinspires.ftc.teamcode.quals.auto;


import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.CommandBase.Commands;
import org.firstinspires.ftc.teamcode.CommandBase.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.quals.RobotQuals;

@Configurable
@Autonomous(name = "Quals Auto Blue")
public class QualsBlueAuto extends LinearOpMode {
    RobotQuals robot;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    Commands commands;
    CommandScheduler commandScheduler;

    //Poses
    private final Pose startPose = new Pose(85, 10.2, Math.toRadians(90));
    private final Pose Pose1 = new Pose(75, 73, Math.toRadians(135));
    //private final Pose Pose2 = new Pose(20, 40, Math.toRadians(0));
    //private final Pose Pose3 = new Pose(40, 60, /*TI*/ Math.toRadians(90));
    /// / private final Pose Pose4 = new Pose(40, 20, Math.toRadians(45)) ;
    /// / private final Pose Pose5 = new Pose(40, 20, Math.toRadians(135));


    private Path scorePreload;
    private PathChain Line1, Line2, Curve3; // Curve4, Line5, Line6;

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

        while(opModeIsActive()){
            // These loop the movements of the robot, these must be called continuously in order to work
            follower.update();
            autonomousPathUpdate();

            // Feedback to Driver Hub for debugging
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

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
                .addPath(new BezierLine(startPose, Pose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), Pose1.getHeading())
                .build();

        /*Line2 = follower.pathBuilder()
                .addPath(new BezierLine(Pose1, Pose2))
                .setLinearHeadingInterpolation(Pose1.getHeading(), Pose2.getHeading())
                .build();

        Curve3 = follower.pathBuilder()
                .addPath(new BezierCurve(Pose2, Pose3, Pose1))
                .setLinearHeadingInterpolation(Pose2.getHeading(), Pose3.getHeading(), Pose1.getHeading())
                .build();

         */
    }

    SequentialCommandGroup auto = new SequentialCommandGroup(
            new InstantCommand(() -> {
                robot.belt.setPower(0.75); //example
            })
    );
}