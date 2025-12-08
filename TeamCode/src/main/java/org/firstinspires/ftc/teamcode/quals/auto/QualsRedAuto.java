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

import org.firstinspires.ftc.teamcode.CommandBase.BeltSubsystem;
import org.firstinspires.ftc.teamcode.CommandBase.Commands;
import org.firstinspires.ftc.teamcode.CommandBase.FlySubsystem;
import org.firstinspires.ftc.teamcode.CommandBase.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.CommandBase.RunBelt;
import org.firstinspires.ftc.teamcode.CommandBase.RunIntake;
import org.firstinspires.ftc.teamcode.CommandBase.ShootMacro;
import org.firstinspires.ftc.teamcode.CommandBase.SortMacro;
import org.firstinspires.ftc.teamcode.CommandBase.SortSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.quals.QualsTeleOp;
import org.firstinspires.ftc.teamcode.quals.RobotQuals;


@Configurable
@Autonomous(name = "Quals Auto Red")
public class QualsRedAuto extends LinearOpMode{
    RobotQuals robot;
    private Follower follower;
    private Timer pathTimer, autoTimer, opmodeTimer;
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
    private final Pose pickupControl1 = new Pose(85, 85);
    private final Pose shootControl = new Pose(90, 90);
    private final Pose pickupControl2 = new Pose(75, 57);
    private final Pose leave = new Pose(72, 55);
    private Path scorePreload;
    private PathChain Line1, Curve2, Line3, Curve4, Curve5, Line6, Curve7, Line8;

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

        }
    }
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
    SortMacro sortMacro = new SortMacro(beltSubsystem, sortSubsystem,intakeSubsystem);
    RunBelt runBelt = new RunBelt(beltSubsystem);
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
                    //TODO: does break go inside the if?
                    break;
                }
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

/*
    //TODO: again, what is this? -emad
    SequentialCommandGroup auto = new SequentialCommandGroup(
            new InstantCommand(() -> {
                robot.belt.setPower(0.75); //example
            })
    );
*/

}