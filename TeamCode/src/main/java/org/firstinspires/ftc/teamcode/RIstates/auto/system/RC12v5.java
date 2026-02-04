package org.firstinspires.ftc.teamcode.RIstates.auto.system;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.RIstates.management.SystemManager;
import org.firstinspires.ftc.teamcode.RIstates.management.fsm.FSM;

import java.util.concurrent.TimeUnit;

@Autonomous()
public class RC12v5 extends OpMode {
    SystemManager manager;
    Timing.Timer flyTimer;
    Timing.Timer advanceTimer;
    Boolean timing;
    boolean following;
    public void autonomousPathUpdate() {
        switch (manager.pathState) {
            case 0:
                if (!following) {
                    manager.follower.followPath(manager.rc12Paths.startToShoot, false);
                    following = true;
                }
                if (!manager.follower.isBusy()) {
                    manager.setPathState(1);
                }
                break;
            case 1:
                if (!flyTimer.isTimerOn() && !timing) {
                    advanceTimer.start();
                    flyTimer.start();
                    timing = true;
                }
                if (flyTimer.isTimerOn() && !flyTimer.done()) {
                    manager.FSM.runNew(FSM.StateName.Shoot);
                }
                if (timing && advanceTimer.done()) {
                    manager.intakeController.shootRun();
                }
                if (timing && flyTimer.done()) {
                    manager.FSM.runNew(FSM.StateName.Norm);
                    timing = false;
                    manager.follower.followPath(manager.rc12Paths.intakeSpike2, false);
                    manager.setPathState(2);
                }
                break;
            case 2:
                manager.FSM.runNew(FSM.StateName.Intake);
                if (!manager.follower.isBusy()) {
                    manager.FSM.runNew(FSM.StateName.Norm);
                    manager.follower.followPath(manager.rc12Paths.openGate, false);
                    manager.setPathState(3);
                }
                break;
            case 3:
                if (!manager.follower.isBusy()) {
                    manager.follower.followPath(manager.rc12Paths.gateToShoot, false);
                    manager.setPathState(4);
                }
                break;
            case 4:
                if (!manager.follower.isBusy()) {
                    manager.setPathState(5);
                }
            case 5:
                if (!flyTimer.isTimerOn() && !timing) {
                    advanceTimer.start();
                    flyTimer.start();
                    timing = true;
                }
                if (flyTimer.isTimerOn() && !flyTimer.done()) {
                    manager.FSM.runNew(FSM.StateName.Shoot);
                }
                if (timing && advanceTimer.done()) {
                    manager.intakeController.shootRun();
                }
                if (timing && flyTimer.done()) {
                    manager.FSM.runNew(FSM.StateName.Norm);
                    manager.follower.followPath(manager.rc12Paths.intakeSpike1, false);
                    timing = false;
                    manager.setPathState(6);
                }
                break;
            case 6:
                manager.FSM.runNew(FSM.StateName.Intake);
                if (!manager.follower.isBusy()) {
                    manager.FSM.runNew(FSM.StateName.Norm);
                    manager.follower.followPath(manager.rc12Paths.spike1toShoot, false);
                    manager.setPathState(7);
                }
            case 7:
                if (!manager.follower.isBusy()) {
                    manager.setPathState(8);
                }
            case 8:
                if (!flyTimer.isTimerOn() && !timing) {
                    advanceTimer.start();
                    flyTimer.start();
                    timing = true;
                }
                if (flyTimer.isTimerOn() && !flyTimer.done()) {
                    manager.FSM.runNew(FSM.StateName.Shoot);
                }
                if (timing && advanceTimer.done()) {
                    manager.intakeController.shootRun();
                }
                if (timing && flyTimer.done()) {
                    manager.FSM.runNew(FSM.StateName.Norm);
                    manager.follower.followPath(manager.rc12Paths.intakeSpike3, false);
                    timing = false;
                    manager.setPathState(9);
                }
                break;
            case 9:
                manager.FSM.runNew(FSM.StateName.Intake);
                if (!manager.follower.isBusy()) {
                    manager.FSM.runNew(FSM.StateName.Norm);
                    manager.follower.followPath(manager.rc12Paths.spike3toShoot, false);
                    manager.setPathState(10);
                }
            case 10:
                if (!flyTimer.isTimerOn() && !timing) {
                    advanceTimer.start();
                    flyTimer.start();
                    timing = true;
                }
                if (flyTimer.isTimerOn() && !flyTimer.done()) {
                    manager.FSM.runNew(FSM.StateName.Shoot);
                }
                if (timing && advanceTimer.done()) {
                    manager.intakeController.shootRun();
                }
                if (timing && flyTimer.done()) {
                    manager.FSM.runNew(FSM.StateName.Norm);
                    manager.follower.followPath(manager.rc12Paths.shootToLeave, false);
                    timing = false;
                    manager.setPathState(11);
                }
                break;
            case 11:
                break;
        }
    }

    @Override
    public void loop() {
        autonomousPathUpdate();
        manager.telemetryM.addData("path state", manager.pathState);
        manager.telemetryM.addData("x", manager.follower.getPose().getX());
        manager.telemetryM.addData("y", manager.follower.getPose().getY());
        manager.telemetryM.addData("heading", manager.follower.getPose().getHeading());
        manager.telemetryM.addData("follower busy?", manager.follower.isBusy());
        manager.autoUpdate();
    }
    @Override
    public void init() {
        manager = new SystemManager(hardwareMap, telemetry, gamepad1, gamepad2, false, false);
        manager.init();
        manager.setAlliance(2);
        manager.opmodeTimer.resetTimer();
        manager.rc12Paths.buildPaths();
        manager.follower.setStartingPose(manager.bf12Paths.farStartPose);
        flyTimer = new Timing.Timer(3500, TimeUnit.MILLISECONDS);
        advanceTimer = new Timing.Timer(500, TimeUnit.MILLISECONDS);
        advanceTimer.pause();
        flyTimer.pause();
        timing = false;
        following = false;
    }
    @Override
    public void init_loop() {
    }
    @Override
    public void start() {
        manager.opmodeTimer.resetTimer();
        manager.setPathState(0);
    }
    @Override
    public void stop() {}
}
