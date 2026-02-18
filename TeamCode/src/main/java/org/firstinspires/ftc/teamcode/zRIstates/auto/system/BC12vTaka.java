package org.firstinspires.ftc.teamcode.zRIstates.auto.system;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.zRIstates.management.SystemManager;
import org.firstinspires.ftc.teamcode.zRIstates.management.fsm.FSM;

import java.util.concurrent.TimeUnit;

@Configurable
@Autonomous()
public class BC12vTaka extends OpMode {
    SystemManager manager;
    Timing.Timer flyTimer;
    Timing.Timer advanceTimer;
    boolean timing;
    boolean following;
    ElapsedTime shootTimer = new ElapsedTime();
    public static double shootTime = 6;
    ElapsedTime intakeTime = new ElapsedTime();
    public static double waitPls = 1;
    private Timing.Timer emadTime;
    private static int emadTimeLength = 1000;

    public void autonomousPathUpdate() {
        switch (manager.pathState) {
            case 0:
                if (!following) {
                    manager.follower.followPath(manager.bc12Paths.startToShoot, false);
                    following = true;
                }
                if (!manager.follower.isBusy()) {
                    manager.setPathState(1);
                }
                break;
            case 1:
                if (!timing) {
                    advanceTimer.start();
                    flyTimer.start();
                    timing = true;
                    manager.FSM.runNew(FSM.StateName.Shoot);
                }
                if (timing && advanceTimer.done()) {
                    manager.intakeController.shootRun();
                }
                if (timing && flyTimer.done()) {
                    manager.FSM.runNew(FSM.StateName.Norm);
                    timing = false;
                    manager.follower.followPath(manager.bc12Paths.intakeSpike2, false);
                    manager.setPathState(2);
                }
                break;
            case 2:
                manager.FSM.runNew(FSM.StateName.Intake);
                if (!manager.follower.isBusy()) {
                    manager.FSM.runNew(FSM.StateName.Norm);
                    manager.follower.followPath(manager.bc12Paths.spike2ToShoot, false);
                    manager.setPathState(3);
                }
                break;
            case 3:
                if (!manager.follower.isBusy()) {
                    manager.follower.followPath(manager.bc12Paths.gateToShoot, false);
                    manager.setPathState(4);
                }
                break;
            case 4:
                if (!manager.follower.isBusy()) {
                    manager.setPathState(5);
                }
            case 5:
                if (!timing) {
                    advanceTimer.start();
                    flyTimer.start();
                    timing = true;
                    manager.FSM.runNew(FSM.StateName.Shoot);
                }
                if (timing && advanceTimer.done()) {
                    manager.intakeController.shootRun();
                }
                if (timing && flyTimer.done()) {
                    manager.FSM.runNew(FSM.StateName.Norm);
                    manager.follower.followPath(manager.bc12Paths.intakeSpike1, false);
                    timing = false;
                    manager.setPathState(6);
                }
                break;
            case 6:
                manager.FSM.runNew(FSM.StateName.Intake);
                if (!manager.follower.isBusy()) {
                    manager.FSM.runNew(FSM.StateName.Norm);
                    manager.follower.followPath(manager.bc12Paths.spike1toShoot, false);
                    manager.setPathState(7);
                }
            case 7:
                if (!manager.follower.isBusy()) {
                    manager.setPathState(8);
                }
            case 8:
                if (!timing) {
                    advanceTimer.start();
                    flyTimer.start();
                    timing = true;
                    manager.FSM.runNew(FSM.StateName.Shoot);
                }
                if (timing && advanceTimer.done()) {
                    manager.intakeController.shootRun();
                }
                if (timing && flyTimer.done()) {
                    manager.FSM.runNew(FSM.StateName.Norm);
                    manager.follower.followPath(manager.bc12Paths.intakeSpike3, false);
                    timing = false;
                    manager.setPathState(9);
                }
                break;
            case 9:
                manager.FSM.runNew(FSM.StateName.Intake);
                if (!manager.follower.isBusy()) {
                    manager.FSM.runNew(FSM.StateName.Norm);
                    manager.follower.followPath(manager.bc12Paths.spike3toShoot, false);
                    manager.setPathState(10);
                }
            case 10:
                if (!timing) {
                    advanceTimer.start();
                    flyTimer.start();
                    timing = true;
                    manager.FSM.runNew(FSM.StateName.Shoot);
                }
                if (timing && advanceTimer.done()) {
                    manager.intakeController.shootRun();
                }
                if (timing && flyTimer.done()) {
                    manager.FSM.runNew(FSM.StateName.Norm);
                    manager.follower.followPath(manager.bc12Paths.shootToLeave, false);
                    timing = false;
                    manager.setPathState(11);
                }
                break;
            case 11:
                break;

        }
    }
    ElapsedTime autoTime = new ElapsedTime();
    public static double abortMission = 27;
    public static boolean fled = false;
    @Override
    public void loop() {
        if(autoTime.seconds() > abortMission && !fled){
            manager.follower.followPath(manager.bc12Paths.abort);
            fled = true;
        }
        else if(autoTime.seconds() > abortMission && fled){
            manager.turret.turnTurret(0);
        }
        else if(autoTime.seconds() < abortMission){
            autonomousPathUpdate();
            manager.turret.trackGoal(manager.shooterHandler.alliance);
        }
        manager.telemetryM.addData("path state", manager.pathState);
        manager.telemetryM.addData("x", manager.follower.getPose().getX());
        manager.telemetryM.addData("y", manager.follower.getPose().getY());
        manager.telemetryM.addData("heading", manager.follower.getPose().getHeading());
        manager.telemetryM.addData("follower busy?", manager.follower.isBusy());
        manager.telemetryM.addData("stopwatch status", autoTime.seconds());
        manager.autoUpdate();
    }
    @Override
    public void init() {
        manager = new SystemManager(hardwareMap, telemetry, gamepad1, gamepad2, false, false);
        manager.init();
        manager.setAlliance(1);
        manager.opmodeTimer.resetTimer();
        manager.bc12Paths.buildPaths();
        manager.follower.setStartingPose(manager.bc12Paths.nearStartPose);
        timing = false;
        following = false;
        emadTime = new Timing.Timer(1000, TimeUnit.MILLISECONDS);
    }
    @Override
    public void init_loop() {
    }
    @Override
    public void start() {
        manager.opmodeTimer.resetTimer();
        autoTime.reset();
        manager.setPathState(0);
    }
    @Override
    public void stop() {
    }
}
