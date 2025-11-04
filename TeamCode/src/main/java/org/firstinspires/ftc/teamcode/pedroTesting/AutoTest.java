package org.firstinspires.ftc.teamcode.pedroTesting;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Example Auto", group = "Examples")
public class AutoTest extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

//    Poses
    private final Pose startPose = new Pose(10, 10, Math.toRadians(0));
    private final Pose Pose1 = new Pose(20, 20, Math.toRadians(0));
    private final Pose Pose2 = new Pose(20, 40, Math.toRadians(0));
    private final Pose Pose3 = new Pose(40, 60, Math.toRadians(90)); //ti
    // private final Pose Pose4 = new Pose(40, 20, Math.toRadians(45));
    // private final Pose Pose5 = new Pose(40, 20, Math.toRadians(135));


    private Path scorePreload;
    private PathChain Line1, Line2, Curve3; // Curve4, Line5, Line6;

    public void buildPaths() {

        Line1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, Pose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), Pose1.getHeading())
                .build();

        Line2 = follower.pathBuilder()
                .addPath(new BezierLine(Pose1, Pose2))
                .setLinearHeadingInterpolation(Pose1.getHeading(), Pose2.getHeading())
                .build();

        Curve3 = follower.pathBuilder()
                .addPath(new BezierCurve(Pose2, Pose3, Pose1))
                .setLinearHeadingInterpolation(Pose2.getHeading(), Pose3.getHeading(), Pose1.getHeading())
                .build();
        }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(Line1);
                setPathState(1);
                break;
            case 1:
                follower.followPath(Line2);
                setPathState(1);
                break;
            case 2:
                follower.followPath(Curve3);
                setPathState(1);
                break;
            case 3:

        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }



    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

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

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        //setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
    }