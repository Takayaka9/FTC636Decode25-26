package org.firstinspires.ftc.teamcode.RIstates.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.states.controllers.subsystems.Hood;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.states.controllers.subsystems.Intake;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.states.controllers.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.states.controllers.ShooterController;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.states.controllers.subsystems.Turret;

//12  ball red far auto with conflict in close shooting position
//all spike marks
//use for if other team has no auto (should be fully revised for comp with more efficient route)
@Autonomous(name = "Red Close 1")
public class RedFar1 extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    //subs
    private Shooter shooter;
    private Hood hood;
    private Turret turret;
    private ShooterController shooterController;
    private Intake intake;
    private final int alliance = 2;
    private final double closeTime = 0;
    private final double farTime = 0;

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
                follower.followPath(fs0, true);
                if (!follower.isBusy()) {
                    shooterController.shootTimeStart();
                    shooterController.shoot(alliance);
                }
                if (shooterController.shootTimeCheck(farTime)) {
                    setPathState(1);
                }
                break;
            case 1:
                follower.followPath(pi1);
                setPathState(2);
                break;
            case 2:
                follower.followPath(i1);
                intake.run();
                setPathState(3);
            case 3:
                follower.followPath(cs1, true);
                if (!follower.isBusy()) {
                    shooterController.shootTimeStart();
                    shooterController.shoot(alliance);
                }
                if (shooterController.shootTimeCheck(farTime)) {
                    setPathState(4);
                }
            case 4:
                follower.followPath(pi2);
                setPathState(5);
                break;
            case 5:
                follower.followPath(i2);
                intake.run();
                setPathState(6);
                break;
            case 6:
                follower.followPath(cs2, true);
                intake.stop();
                if (!follower.isBusy()) {
                    shooterController.shootTimeStart();
                    shooterController.shoot(alliance);
                }
                if (shooterController.shootTimeCheck(farTime)) {
                    setPathState(7);
                }
                break;
            case 7:
                follower.followPath(pi3);
                setPathState(8);
                break;
            case 8:
                follower.followPath(i3);
                intake.run();
                setPathState(9);
                break;
            case 9:
                follower.followPath(fs3, true);
                intake.stop();
                if (!follower.isBusy()) {
                    shooterController.shootTimeStart();
                    shooterController.shoot(alliance);
                }
                if (shooterController.shootTimeCheck(farTime)) {
                    setPathState(10);
                }
                break;
            case 10:
                follower.followPath(l);
                break;
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
        //subs
        shooter = new Shooter(hardwareMap, "flyRight", "flyLeft");
        hood = new Hood(hardwareMap, "servo");
        turret = new Turret(hardwareMap, "turret");
        shooterController = new ShooterController(shooter, hood, turret, follower);
        intake = new Intake(hardwareMap, "intake");
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