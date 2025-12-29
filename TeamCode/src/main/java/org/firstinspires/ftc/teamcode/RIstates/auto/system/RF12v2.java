package org.firstinspires.ftc.teamcode.RIstates.auto.system;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.RIstates.management.SystemManager;

// first command system auto (does not use command system FSM)
@Autonomous(name = "RF12v2", group = "RF12v2")
public class RF12v2 extends OpMode {
    SystemManager manager;
    HardwareMap hardwareMap;

    public void autonomousPathUpdate() {
        switch (manager.pathState) {
            case 0:
                manager.follower.followPath(manager.rf12Paths.fs0, true);
                if (!manager.follower.isBusy()) {
                    manager.shooterController.shoot();
                    if (!manager.shooterController.shooterRunning) {
                        manager.setPathState(1);
                    }
                }
                break;
            case 1:
                manager.follower.followPath(manager.rf12Paths.pi1);
                if (!manager.follower.isBusy()) {
                    manager.setPathState(2);
                }
            case 2:
                manager.intake.run();
                manager.follower.followPath(manager.rf12Paths.i1);
                if (!manager.follower.isBusy()) {
                    manager.setPathState(3);
                }
                break;
            case 3:
                manager.follower.followPath(manager.rf12Paths.cs1, true);
                if (!manager.follower.isBusy()) {
                    manager.shooterController.shoot();
                    if (!manager.shooterController.shooterRunning) {
                        manager.setPathState(4);
                    }
                }
                break;
            case 4:
                manager.follower.followPath(manager.rf12Paths.pi2);
                if (!manager.follower.isBusy()) {
                    manager.setPathState(5);
                }
            case 5:
                manager.intake.run();
                manager.follower.followPath(manager.rf12Paths.i2);
                if (!manager.follower.isBusy()) {
                    manager.setPathState(6);
                }
                break;
            case 6:
                manager.follower.followPath(manager.rf12Paths.cs2, true);
                if (!manager.follower.isBusy()) {
                    manager.shooterController.shoot();
                    if (!manager.shooterController.shooterRunning) {
                        manager.setPathState(6);
                    }
                }
            case 7:
                manager.follower.followPath(manager.rf12Paths.pi3);
                if (!manager.follower.isBusy()) {
                    manager.setPathState(8);
                }
            case 8:
                manager.intake.run();
                manager.follower.followPath(manager.rf12Paths.i3);
                if (!manager.follower.isBusy()) {
                    manager.setPathState(9);
                }
                break;
            case 9:
                manager.follower.followPath(manager.rf12Paths.fs3, true);
                if (!manager.follower.isBusy()) {
                    manager.shooterController.shoot();
                    if (!manager.shooterController.shooterRunning) {
                        manager.setPathState(10);
                    }
                }
                break;
            case 10:
                manager.follower.followPath(manager.rf12Paths.l);
                break;

        }
    }





///default pedro requirements:
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
        manager = new SystemManager(hardwareMap, gamepad1, gamepad2, false);
        manager.init();
        manager.setAlliance(1);
        manager.opmodeTimer.resetTimer();
        manager.rf12Paths.buildPaths();
        manager.follower.setStartingPose(manager.poseLib.farStartPose);
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

