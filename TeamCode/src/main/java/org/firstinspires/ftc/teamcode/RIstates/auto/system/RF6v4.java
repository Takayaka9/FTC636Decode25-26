package org.firstinspires.ftc.teamcode.RIstates.auto.system;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.RIstates.management.SystemManager;
import org.firstinspires.ftc.teamcode.RIstates.management.fsm.FSM;

import java.util.concurrent.TimeUnit;

@Autonomous()
public class RF6v4 extends OpMode {
    SystemManager manager;
    Timing.Timer timer;
    Boolean timing;
    boolean following;
    public void autonomousPathUpdate() {
        switch (manager.pathState) {
            case 0:
                if (!following) {
                    manager.follower.followPath(manager.rf6Paths.startToShoot, false);
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
                    manager.follower.followPath(manager.rf6Paths.intakeSpike3, false);
                    manager.setPathState(2);
                }
                break;
            case 2:
                manager.FSM.runNew(FSM.StateName.Intake);
                if (!manager.follower.isBusy()) {
                    manager.FSM.runNew(FSM.StateName.Norm);
                    manager.follower.followPath(manager.rf6Paths.spike3toShoot, false);
                    manager.setPathState(3);
                }
                break;
            case 3:
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
                    manager.follower.followPath(manager.rf6Paths.startToShoot, false);
                    manager.setPathState(4);
                }
                break;
            case 4:
                if (!manager.follower.isBusy()) {
                    manager.follower.followPath(manager.rf6Paths.shootToLeave, false);
                    manager.setPathState(5);
                }
                break;
            case 5:
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
        manager.rf6Paths.buildPaths();
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
