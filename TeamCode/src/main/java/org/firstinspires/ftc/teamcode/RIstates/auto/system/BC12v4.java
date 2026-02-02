package org.firstinspires.ftc.teamcode.RIstates.auto.system;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.RIstates.management.SystemManager;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Utilities.timers.GenericTime;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Utilities.timers.SolversTiming;
import org.firstinspires.ftc.teamcode.RIstates.management.fsm.FSM;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "RC12v4", group = "RC12v4")
public class BC12v4 extends OpMode {
    SystemManager manager;
    HardwareMap hardwareMap;
    Timing.Timer timer;
    Boolean timing;
    boolean following;
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
                if (!timer.isTimerOn() && !timing) {
                    timer.start();
                    timing = true;
                }
                if (timer.isTimerOn() && !timer.done()) {
                    manager.FSM.runNew(FSM.StateName.Shoot);
                }
                if (timing && timer.done()) {
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
                    manager.follower.followPath(manager.bc12Paths.openGate, false);
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
                if (!timer.isTimerOn() && !timing) {
                    timer.start();
                    timing = true;
                }
                if (timer.isTimerOn() && !timer.done()) {
                    manager.FSM.runNew(FSM.StateName.Shoot);
                }
                if (timing && timer.done()) {
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
                if (!timer.isTimerOn() && !timing) {
                    timer.start();
                    timing = true;
                }
                if (timer.isTimerOn() && !timer.done()) {
                    manager.FSM.runNew(FSM.StateName.Shoot);
                }
                if (timing && timer.done()) {
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
                if (!timer.isTimerOn() && !timing) {
                    timer.start();
                    timing = true;
                }
                if (timer.isTimerOn() && !timer.done()) {
                    manager.FSM.runNew(FSM.StateName.Shoot);
                }
                if (timing && timer.done()) {
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
        manager.bc12Paths.buildPaths();
        manager.follower.setStartingPose(manager.bf12Paths.farStartPose);
        timer = new Timing.Timer(3500, TimeUnit.MILLISECONDS);
        timer.pause();
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
