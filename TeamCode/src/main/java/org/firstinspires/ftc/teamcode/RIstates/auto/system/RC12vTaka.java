package org.firstinspires.ftc.teamcode.RIstates.auto.system;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.RIstates.management.SystemManager;
import org.firstinspires.ftc.teamcode.RIstates.management.fsm.FSM;
import org.firstinspires.ftc.teamcode.RIstates.management.pedro.utils.PoseStorage;

import java.util.concurrent.TimeUnit;
@Autonomous()
public class RC12vTaka extends OpMode {
    SystemManager manager;
    Timing.Timer flyTimer;
    Timing.Timer advanceTimer;
    boolean timing;
    boolean following;
    ElapsedTime shootTimer = new ElapsedTime();
    public static double shootTime = 5;
    public void autonomousPathUpdate() {
        switch (manager.pathState) {
            case 0:
                if (!following) {
                    manager.follower.followPath(manager.rc12Paths.startToShoot);
                    following = true;
                }
                if (!manager.follower.isBusy()) {
                    shootTimer.reset();
                    manager.setPathState(1);
                }
                break;
            case 1:
                if (shootTimer.seconds() < shootTime) {
                    manager.shooterHandler.shoot();
                    manager.shooterHandler.autoShoot();
                }
                else if(shootTimer.seconds() > shootTime){
                    manager.shooterHandler.off();
                    manager.follower.followPath(manager.rc12Paths.intakeSpike2);
                    manager.setPathState(2);
                }
                break;
            case 2:
                manager.intakeController.run();
                if (!manager.follower.isBusy()) {
                    manager.FSM.runNew(FSM.StateName.Norm);
                    manager.intakeController.stop();
                    manager.follower.followPath(manager.rc12Paths.openGate);
                    manager.setPathState(3);
                }
                break;
            case 3:
                if (!manager.follower.isBusy()) {
                    manager.follower.followPath(manager.rc12Paths.gateToShoot);
                    manager.setPathState(4);
                }
                break;
            case 4:
                if (!manager.follower.isBusy()) {
                    shootTimer.reset();
                    manager.setPathState(5);
                }
                break;
            case 5:
                if (shootTimer.seconds() < shootTime) {
                    manager.shooterHandler.shoot();
                    manager.shooterHandler.autoShoot();
                }
                else if(shootTimer.seconds() > shootTime){
                    manager.shooterHandler.off();
                    manager.follower.followPath(manager.rc12Paths.intakeSpike1);
                    manager.setPathState(6);
                }
                break;
            case 6:
                manager.intakeController.run();
                if (!manager.follower.isBusy()) {
                    manager.FSM.runNew(FSM.StateName.Norm);
                    manager.intakeController.stop();
                    manager.follower.followPath(manager.rc12Paths.spike1toShoot);
                    manager.setPathState(7);
                }
                break;
            case 7:
                if (!manager.follower.isBusy()) {
                    shootTimer.reset();
                    manager.setPathState(8);
                }
                break;
            case 8:
                if (shootTimer.seconds() < shootTime) {
                    manager.shooterHandler.shoot();
                    manager.shooterHandler.autoShoot();
                }
                else if(shootTimer.seconds() > shootTime){
                    manager.shooterHandler.off();
                    manager.follower.followPath(manager.rc12Paths.intakeSpike3);
                    manager.setPathState(9);
                }
                break;
            case 9:
                manager.intakeController.run();
                if (!manager.follower.isBusy()) {
                    manager.FSM.runNew(FSM.StateName.Norm);
                    manager.intakeController.stop();
                    manager.follower.followPath(manager.rc12Paths.spike3toShoot);
                    manager.setPathState(12);
                }
                break;
            case 12:
                if(!manager.follower.isBusy()){
                    shootTimer.reset();
                    manager.setPathState(10);
                }
                break;
            case 10:
                if (shootTimer.seconds() < shootTime) {
                    manager.shooterHandler.shoot();
                    manager.shooterHandler.autoShoot();
                }
                else if(shootTimer.seconds() > shootTime){
                    manager.shooterHandler.off();
                    manager.intakeController.stop();
                    manager.FSM.runNew(FSM.StateName.Norm);
                    manager.follower.followPath(manager.rc12Paths.shootToLeave);
                    manager.setPathState(11);
                }
                break;
            case 11:
                break;

        }
    }
    ElapsedTime autoTime = new ElapsedTime();
    public static double abortMission = 28;
    public static boolean fled = false;
    @Override
    public void loop() {
        if(autoTime.seconds() > abortMission && !fled){
            manager.follower.followPath(manager.rc12Paths.abort);
            fled = true;
        }
        else if(autoTime.seconds() > abortMission && fled){
            manager.turret.turnTurret(0);
        }
        else{
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
        manager.setAlliance(2);
        manager.opmodeTimer.resetTimer();
        manager.rc12Paths.buildPaths();
        manager.follower.setStartingPose(manager.rc12Paths.nearStartPose);
        timing = false;
        following = false;
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
