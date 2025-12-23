package org.firstinspires.ftc.teamcode.states.auto;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red Close 1")
public class BlueClose1 extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    //    Poses
    private final Pose startPose = new Pose(24, 129, 143);
    private final Pose shootPose = new Pose(47,96,127);
    private final Pose preIntake1 = new Pose(44, 84, 180);
    private final Pose intake1 = new Pose(18, 84, 180);
    private final Pose preIntake2 = new Pose(44, 58, 180);
    private final Pose intake2 = new Pose(11, 58, 180);
    private final Pose shootControl2 = new Pose(54, 58, 180);
    private final Pose leave = new Pose(24, 96, 127);

    //    Path Initializing
    private PathChain s0, pi1, i1, s1, pi2, i2, s2, l;

    public void buildPaths() {
        s0 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        pi1 = follower.pathBuilder().addPath(new BezierLine(shootPose, preIntake1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), preIntake1.getHeading())
                .build();

        i1 = follower.pathBuilder().addPath(new BezierLine(preIntake1, intake1))
                .setLinearHeadingInterpolation(preIntake1.getHeading(), intake1.getHeading())
                .build();

        s1 = follower.pathBuilder().addPath(new BezierLine(intake1, shootPose))
                .setLinearHeadingInterpolation(intake1.getHeading(), shootPose.getHeading())
                .build();

        pi2 = follower.pathBuilder().addPath(new BezierLine(shootPose, preIntake2))
                .setLinearHeadingInterpolation(shootPose.getHeading(), preIntake2.getHeading())
                .build();

        i2 = follower.pathBuilder().addPath(new BezierLine(preIntake2, intake2))
                .setLinearHeadingInterpolation(preIntake2.getHeading(), intake2.getHeading())
                .build();

        s2 = follower.pathBuilder().addPath(new BezierLine(intake2, shootPose))
                .setLinearHeadingInterpolation(intake2.getHeading(), shootPose.getHeading())
                .build();

        l = follower.pathBuilder().addPath(new BezierLine(shootPose, leave))
                .setLinearHeadingInterpolation(shootPose.getHeading(), leave.getHeading())
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