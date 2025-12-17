package org.firstinspires.ftc.teamcode.zquals.auto.trash;

import com.arcrobotics.ftclib.command.CommandScheduler;
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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.zCommandBase.BeltSubsystem;
import org.firstinspires.ftc.teamcode.zCommandBase.Commands;
import org.firstinspires.ftc.teamcode.zCommandBase.FlySubsystem;
import org.firstinspires.ftc.teamcode.zCommandBase.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.zCommandBase.RunIntake;
import org.firstinspires.ftc.teamcode.zCommandBase.ShootMacro;
import org.firstinspires.ftc.teamcode.zCommandBase.SortSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.zquals.RobotQuals;

@Disabled
@Configurable
@Autonomous(name = "Quals Auto Blue")
public class QualsBlueAuto extends OpMode{
    RobotQuals robot;
    private Follower follower;
    private Timer pathTimer, autoTimer, opmodeTimer;
    private int pathState;
    Commands commands;
    CommandScheduler commandScheduler;


    //Poses
    private final Pose startPose = new Pose(27, 129, Math.toRadians(143));
    private final Pose shootPose = new Pose(69, 75, Math.toRadians(135));
    private final Pose prePickup1 = new Pose(44, 80, Math.toRadians(180));
    private final Pose Pickup1 = new Pose(24, 80, Math.toRadians(180));
    //back to shoot pose
    private final Pose prePickup2 = new Pose(44, 57, Math.toRadians(180));
    private final Pose Pickup2 = new Pose(24, 57, Math.toRadians(180));
    //back to shoot pose again

    //control point poses
    private final Pose pickupControl1 = new Pose(59, 85);
    private final Pose shootControl = new Pose(54, 90);
    private final Pose pickupControl2 = new Pose(69, 57);
    private final Pose leave = new Pose(72, 55);
    private Path scorePreload;
    private PathChain Line1, Curve2, Line3, Curve4, Curve5, Line6, Curve7, Line8;

    /*
        //TODO: what is this? -emad
        private void runPath(PathChain p) {
            follower.followPath(p);
            while (opModeIsActive() && follower.isBusy()) {
                follower.update();
            }
        }
    */
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

        Curve7 = follower.pathBuilder()
                .addPath(new BezierCurve(Pickup2, shootControl, shootPose))
                .setLinearHeadingInterpolation(Pickup2.getHeading(), shootPose.getHeading())
                .build();
        Line8 = follower.pathBuilder()
                .addPath(new BezierLine(prePickup2, Pickup2))
                .setConstantHeadingInterpolation(Pickup2.getHeading())
                .build();
    }

    // USE COMMAND SYSTEM IN THE NEXT BIT I HAVEN'T DONE IT YET
    FlySubsystem flySubsystem;
    BeltSubsystem beltSubsystem;
    IntakeSubsystem intakeSubsystem;
    SortSubsystem sortSubsystem;
    ShootMacro shootMacro = new ShootMacro(beltSubsystem, sortSubsystem,flySubsystem,intakeSubsystem);
    RunIntake runIntake = new RunIntake(intakeSubsystem);

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(Line1);
                setPathState(1);
                break;
            case 1:
                shootMacro.schedule();
                if (shootMacro.isFinished == true) {
                    shootMacro.end();
                    follower.followPath(Curve2);
                    setPathState(2);
                }
                break;
            case 2:
                follower.followPath(Line3);
                setPathState(3);
                if (!follower.isBusy()){
                    runIntake.schedule();
                }
                break;
            case 3:
                follower.followPath(Curve4);
                setPathState(4);
                break;
            case 4:
                runIntake.end();
                if (!follower.isBusy()) {
                    shootMacro.schedule();
                }
                if (shootMacro.isFinished == true){
                    shootMacro.end();
                    follower.followPath(Curve5);
                    setPathState(5);
                }
                if (!follower.isBusy()) {
                    runIntake.schedule();
                    break;
                }
            case 5:
                follower.followPath(Line6);
                setPathState(6);
                break;
            case 6:
                runIntake.end();
                follower.followPath(Curve7);
                if (!follower.isBusy()) {
                    shootMacro.schedule();
                }
                if (shootMacro.isFinished == true) {
                    shootMacro.end();
                    setPathState(7);
                }
                break;
            case 7:
                follower.followPath(Line8);
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
        CommandScheduler.getInstance().run();
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
/*
    //TODO: again, what is this? -emad
    SequentialCommandGroup auto = new SequentialCommandGroup(
            new InstantCommand(() -> {
                robot.belt.setPower(0.75); //example
            })
    );
*/

}