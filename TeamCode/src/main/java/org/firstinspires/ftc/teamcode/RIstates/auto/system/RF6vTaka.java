package org.firstinspires.ftc.teamcode.RIstates.auto.system;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.RIstates.management.SystemManager;
import org.firstinspires.ftc.teamcode.RIstates.management.fsm.FSM;

import java.util.concurrent.TimeUnit;
@Configurable
@Autonomous()
public class RF6vTaka extends OpMode {
    SystemManager manager;
    boolean timing;
    boolean following;
    ElapsedTime shootTimer = new ElapsedTime();
    public static double shootTime = 12;
    ElapsedTime intakeTime = new ElapsedTime();
    Timing.Timer emadTime;
    private static int emadTimerLength = 1000;
    public static double waitPls = 2;
    public void autonomousPathUpdate() {
        switch (manager.pathState) {
            case 0:
                if (!following) {
                    manager.follower.followPath(manager.rf6Paths.startToShoot);
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
                    emadTime.start();
                }
                else if (!emadTime.done()) {
                    manager.shooterHandler.off();
                    manager.intakeController.reverse();
                }
                else if(shootTimer.seconds() > shootTime){
                    manager.shooterHandler.off();
                    manager.intakeController.stop();
                    manager.follower.followPath(manager.rf6Paths.preIntakeSpike3, 0.5, true);
                    manager.setPathState(67);
                }
                break;
            case 67:
                if(!manager.follower.isBusy()){
                    manager.intakeController.run();
                    manager.follower.followPath(manager.rf6Paths.intakeSpike3, 0.5, true);
                    intakeTime.reset();
                    manager.setPathState(2);
                }
            case 2:
                manager.intakeController.run();
                if (!manager.follower.isBusy() && intakeTime.seconds() > waitPls) {
                    manager.FSM.runNew(FSM.StateName.Norm);
                    manager.follower.followPath(manager.rf6Paths.spike3toShoot);
                    shootTimer.reset();
                    manager.setPathState(3);
                }
                break;
            case 3:
                if (shootTimer.seconds() < shootTime) {
                    manager.shooterHandler.shoot();
                    manager.shooterHandler.autoShoot();
                    emadTime.start();
                }
                else if (!emadTime.done()) {
                    manager.shooterHandler.off();
                    manager.intakeController.reverse();
                }
                else if(shootTimer.seconds() > shootTime){
                    manager.shooterHandler.off();
                    manager.intakeController.stop();
                    manager.setPathState(4);
                }
                break;
            case 4:
                if (!manager.follower.isBusy()) {
                    manager.follower.followPath(manager.rf6Paths.shootToLeave);
                    manager.setPathState(5);
                }
                break;
            case 5:
                break;
        }
    }
    ElapsedTime autoTime = new ElapsedTime();
    public static double abortMission = 28;
    public static boolean fled = false;
    @Override
    public void loop() {
        if(autoTime.seconds() > abortMission && !fled){
            manager.follower.followPath(manager.rf6Paths.abort);
            fled = true;
        }
        else if(autoTime.seconds() > abortMission && fled){
            manager.turret.turnTurret(0);
        }
        else{
            autonomousPathUpdate();
            manager.turret.trackGoal(manager.shooterHandler.alliance);
        }
        //autonomousPathUpdate();
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
        manager.follower.setStartingPose(manager.rf6Paths.farStartPose);
        timing = false;
        following = false;
        emadTime = new Timing.Timer(emadTimerLength, TimeUnit.MILLISECONDS);
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
