package org.firstinspires.ftc.teamcode.RIstates.auto.system;

import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RIstates.management.SystemManager;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "RF12v2", group = "RF12v2")
public class RF12v2 extends OpMode {
    SystemManager manager;
    HardwareMap hardwareMap;

    public void autonomousPathUpdate() {
        switch (manager.pathState) {
            case 0:
                manager.follower.followPath(manager.rf12Paths.fs0, true);
                if (!manager.follower.isBusy()) {
                    manager.shooterController.shootTimeStart();
                    manager.shooterController.shoot();

                }

                break;
        }
    }












///default pedro requirements:
    @Override
    public void loop() {
        manager.follower.update();
        //autonomousPathUpdate();

        manager.telemetryM.addData("path state", manager.pathState);
        manager.telemetryM.addData("x", manager.follower.getPose().getX());
        manager.telemetryM.addData("y", manager.follower.getPose().getY());
        manager.telemetryM.addData("heading", manager.follower.getPose().getHeading());
        manager.telemetryM.update();
    }
    @Override
    public void init() {
        manager.pathTimer = new Timer();
        manager.opmodeTimer = new Timer();
        manager.opmodeTimer.resetTimer();
        manager.rf12Paths.buildPaths();
        manager.follower.setStartingPose(manager.poseLib.farStartPose);
    }
    @Override
    public void init_loop() {}
    @Override
    public void start() {
        manager.opmodeTimer.resetTimer();
        manager.setPathState(0);
    }
    @Override
    public void stop() {}
}

