package org.firstinspires.ftc.teamcode.states.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

//12  ball red far auto with conflict in close shooting position
//all spike marks
//use for if other team has no auto (should be fully revised for comp with more efficient route)
@Autonomous(name = "Red Close 1")
public class RedFar1 extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    //    Poses
    private final Pose startPose = new Pose(87.5, 8.7, 90);
    private final Pose closeShootPose = new Pose(97,96,53);
    private final Pose farShootPose = new Pose(87.5, 15, 70);
    private final Pose preIntake1 = new Pose(100, 62, 0);
    private final Pose intake1 = new Pose(130, 62, 0);
    private final Pose preIntake2 = new Pose(100, 84, 0);
    private final Pose intake2 = new Pose(125, 84, 15);
    private final Pose preIntake3 = new Pose(100, 35, 0);
    private final Pose intake3 = new Pose(133, 35, 0);
    private final Pose leave = new Pose(91.5, 30, 70);

    //    Path Initializing
    private PathChain fs0, pi1, i1, cs1, pi2, i2, cs2, pi3, i3, fs3, l;

    public void buildPaths() {
        fs0 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, farShootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), farShootPose.getHeading())
                .build();

        pi1 = follower.pathBuilder().addPath(new BezierLine(farShootPose, preIntake1))
                .setLinearHeadingInterpolation(farShootPose.getHeading(), preIntake1.getHeading())
                .build();

        i1 = follower.pathBuilder().addPath(new BezierLine(preIntake1, intake1))
                .setLinearHeadingInterpolation(preIntake1.getHeading(), intake1.getHeading())
                .build();

        cs1 = follower.pathBuilder().addPath(new BezierLine(intake1, closeShootPose))
                .setLinearHeadingInterpolation(intake1.getHeading(), closeShootPose.getHeading())
                .build();

        pi2 = follower.pathBuilder().addPath(new BezierLine(closeShootPose, preIntake2))
                .setLinearHeadingInterpolation(closeShootPose.getHeading(), preIntake2.getHeading())
                .build();

        i2 = follower.pathBuilder().addPath(new BezierLine(preIntake2, intake2))
                .setLinearHeadingInterpolation(preIntake2.getHeading(), intake2.getHeading())
                .build();

        cs2 = follower.pathBuilder().addPath(new BezierLine(intake2, closeShootPose))
                .setLinearHeadingInterpolation(intake2.getHeading(), closeShootPose.getHeading())
                .build();

        pi3 = follower.pathBuilder().addPath(new BezierLine(closeShootPose, preIntake3))
                .setLinearHeadingInterpolation(closeShootPose.getHeading(), preIntake3.getHeading())
                .build();

        i3 = follower.pathBuilder().addPath(new BezierLine(preIntake3, intake3))
                .setLinearHeadingInterpolation(preIntake3.getHeading(), intake3.getHeading())
                .build();

        fs3 = follower.pathBuilder()
                .addPath(new BezierLine(intake3, farShootPose))
                .setLinearHeadingInterpolation(intake3.getHeading(), farShootPose.getHeading())
                .build();

        l = follower.pathBuilder().addPath(new BezierLine(closeShootPose, leave))
                .setLinearHeadingInterpolation(closeShootPose.getHeading(), leave.getHeading())
                .build();
    }

    //    path state FSM
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:

        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

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
        setPathState(0);
    }
    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}

}