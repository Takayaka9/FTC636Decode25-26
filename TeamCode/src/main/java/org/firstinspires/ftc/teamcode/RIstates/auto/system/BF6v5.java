//package org.firstinspires.ftc.teamcode.RIstates.auto.system;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.seattlesolvers.solverslib.util.Timing;
//
//import org.firstinspires.ftc.teamcode.RIstates.management.SystemManager;
//import org.firstinspires.ftc.teamcode.RIstates.management.fsm.FSM;
//
//import java.util.concurrent.TimeUnit;
//
//@Autonomous()
//public class BF6v5 extends OpMode {
//    SystemManager manager;
//    boolean timing;
//    boolean following;
//    Tim.fly
//    public void autonomousPathUpdate() {
//        switch (manager.pathState) {
//            case 0:
//                if (!following) {
//                    manager.follower.followPath(manager.bf6Paths.startToShoot);
//                    following = true;
//                }
//                if (!manager.follower.isBusy()) {
//                    manager.setPathState(1);
//                }
//                break;
//            case 1:
//                if (!flyTimer.isTimerOn() && !timing) {
//                    advanceTimer.start();
//                    flyTimer.start();
//                    timing = true;
//                }
//                if (flyTimer.isTimerOn() && !flyTimer.done()) {
//                    manager.FSM.runNew(FSM.StateName.Shoot);
//                }
//                if (timing && advanceTimer.done()) {
//                    manager.intakeController.shootRun();
//                }
//                if (timing && flyTimer.done()) {
//                    manager.FSM.runNew(FSM.StateName.Norm);
//                    timing = false;
//                    manager.follower.followPath(manager.bf6Paths.intakeSpike3);
//                    manager.setPathState(2);
//                }
//                break;
//            case 2:
//                manager.FSM.runNew(FSM.StateName.Intake);
//                if (!manager.follower.isBusy()) {
//                    manager.FSM.runNew(FSM.StateName.Norm);
//                    manager.follower.followPath(manager.bf6Paths.spike3toShoot);
//                    manager.setPathState(3);
//                }
//                break;
//            case 3:
//                if (!flyTimer.isTimerOn() && !timing) {
//                    advanceTimer.start();
//                    flyTimer.start();
//                    timing = true;
//                }
//                if (flyTimer.isTimerOn() && !flyTimer.done()) {
//                    manager.FSM.runNew(FSM.StateName.Shoot);
//                }
//                if (timing && advanceTimer.done()) {
//                    manager.intakeController.shootRun();
//                }
//                if (timing && flyTimer.done()) {
//                    manager.FSM.runNew(FSM.StateName.Norm);
//                    timing = false;
//                    manager.follower.followPath(manager.bf6Paths.startToShoot);
//                    manager.setPathState(4);
//                }
//                break;
//            case 4:
//                if (!manager.follower.isBusy()) {
//                    manager.follower.followPath(manager.bf6Paths.shootToLeave);
//                    manager.setPathState(5);
//                }
//                break;
//            case 5:
//                break;
//        }
//    }
//    ElapsedTime autoTime = new ElapsedTime();
//    public static double abortMission = 28;
//    public static boolean fled = false;
//    @Override
//    public void loop() {
//        if(autoTime.seconds() > abortMission && !fled){
//            manager.follower.followPath(manager.rc12Paths.abort);
//            fled = true;
//        }
//        else if(autoTime.seconds() > abortMission && fled){
//            manager.turret.turnTurret(0);
//        }
//        else{
//            autonomousPathUpdate();
//            manager.turret.trackGoal(manager.shooterHandler.alliance);
//        }
//        //autonomousPathUpdate();
//        manager.telemetryM.addData("path state", manager.pathState);
//        manager.telemetryM.addData("x", manager.follower.getPose().getX());
//        manager.telemetryM.addData("y", manager.follower.getPose().getY());
//        manager.telemetryM.addData("heading", manager.follower.getPose().getHeading());
//        manager.telemetryM.addData("follower busy?", manager.follower.isBusy());
//        manager.autoUpdate();
//    }
//    @Override
//    public void init() {
//        manager = new SystemManager(hardwareMap, telemetry, gamepad1, gamepad2, false, false);
//        manager.init();
//        manager.setAlliance(1);
//        manager.opmodeTimer.resetTimer();
//        manager.bf6Paths.buildPaths();
//        manager.follower.setStartingPose(manager.bf12Paths.farStartPose);
//        flyTimer = new Timing.Timer(3500, TimeUnit.MILLISECONDS);
//        advanceTimer = new Timing.Timer(500, TimeUnit.MILLISECONDS);
//        flyTimer.pause();
//        advanceTimer.pause();
//        timing = false;
//        following = false;
//    }
//    @Override
//    public void init_loop() {
//    }
//    @Override
//    public void start() {
//        manager.opmodeTimer.resetTimer();
//        manager.setPathState(0);
//    }
//    @Override
//    public void stop() {}
//}
