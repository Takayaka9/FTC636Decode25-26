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
    private PathChain fs0, pi1, i1, cs1, pi2, i2, cs2, pi3, i3, fs3, l;







    public void setPathState(int pState) {
        manager.pathState = pState;
        manager.pathTimer.resetTimer();
    }

    @Override
    public void loop() {
//        manager = new SystemManager(hardwareMap, gamepad1, gamepad2, false);
//
//        manager.follower.update();
//        autonomousPathUpdate();
//
//        telemetry.addData("path state", pathState);
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", follower.getPose().getHeading());
//        telemetry.update();
    }
    @Override
    public void init() {
//        manager.pathTimer = new Timer();
//        manager.opmodeTimer = new Timer();
//        manager.opmodeTimer.resetTimer();
//        buildPaths();
//        manager.follower.setStartingPose(startPose);
    }
    @Override
    public void init_loop() {}
    @Override
    public void start() {
        manager.opmodeTimer.resetTimer();
        setPathState(0);
    }
    @Override
    public void stop() {}

}

