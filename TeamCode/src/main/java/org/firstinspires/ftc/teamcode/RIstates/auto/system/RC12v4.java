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
public class RC12v4 extends OpMode {
    SystemManager manager;
    HardwareMap hardwareMap;
    Timing.Timer timer;
    Boolean timing;
    public void autonomousPathUpdate() {
        switch (manager.pathState) {
            case 0:
                manager.follower.followPath(manager.rc12Paths.startToShoot, true);
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
                    manager.setPathState(2);
                    timing = false;
                }


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
        timer = new Timing.Timer(3500, TimeUnit.MILLISECONDS);
        timer.pause();
        timing = false;
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
