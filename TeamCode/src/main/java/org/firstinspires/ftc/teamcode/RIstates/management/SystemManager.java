package org.firstinspires.ftc.teamcode.RIstates.management;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.HoodController;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.servo.LiftServo;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.servo.Transfer;
import org.firstinspires.ftc.teamcode.RIstates.management.pedro.BC12Paths;
import org.firstinspires.ftc.teamcode.RIstates.management.pedro.BF12Paths;
import org.firstinspires.ftc.teamcode.RIstates.management.pedro.BF6Paths;
import org.firstinspires.ftc.teamcode.RIstates.management.pedro.RC12Paths;
import org.firstinspires.ftc.teamcode.RIstates.management.pedro.RF6Paths;
import org.firstinspires.ftc.teamcode.RIstates.management.pedro.utils.PoseStorage;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.servo.base.GamepadServoImplEx;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.servo.GateServo;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.LimelightHandler;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Controller;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Limelight.LimelightController;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Turret;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.color.IntakeDistanceSensor;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.ShooterHandler;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.servo.IntakeController;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Light.LightController;
import org.firstinspires.ftc.teamcode.RIstates.management.pedro.RF12Paths;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.TeleOpHandler;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.color.TurretSensor;
import org.firstinspires.ftc.teamcode.RIstates.management.fsm.FSM;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Intake;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Shooter;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Drive.TeleOpDriveController;

public class SystemManager {
    public final Follower follower;
    public final TelemetryManager telemetryM;
    public final Turret turret;
    public final HoodController hoodController;
    public final Shooter shooter;
    public final Intake intake;
    public final LiftServo liftServo;
    public final GamepadServoImplEx gateServo;
//    public final IntakeSensor intakeSensor;
    public final IntakeDistanceSensor intakeDistanceSensor;
    public final TurretSensor turretSensor;
    public final IntakeController intakeController;
    public final ShooterHandler shooterHandler;
    //public final BallController ballController;
    public final Controller driveController;
    public final LightController lightController;
    public final LimelightController limelightController;
    public final LimelightHandler limelightHandler;
    public final PoseStorage poseStorage;
    public final Transfer transfer;

    public Gamepad gamepad1;
    public Gamepad gamepad2;


    public Timer pathTimer, actionTimer, opmodeTimer;
    public int pathState;
    public final RF12Paths rf12Paths;
    public final BF12Paths bf12Paths;
    public final BC12Paths bc12Paths;
    public final RC12Paths rc12Paths;
    public final RF6Paths rf6Paths;
    public final BF6Paths bf6Paths;

    private final Telemetry telemetry;

    private boolean isTeleop;


    public SystemManager(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, boolean isTeleOp, boolean testing) {
        ///DO NOT CHANGE THE INIT ORDER, add new stuff in it's respective place to avoid NullPointer

        //dependencies
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        isTeleop = isTeleOp;
        pathState = 0;
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        this.telemetry = telemetry;
        follower = Constants.createFollower(hardwareMap);




        //auto dependencies
        poseStorage = new PoseStorage();
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();

        //subsystems
        shooter = new Shooter(hardwareMap, "sr", "sl");

        turret = new Turret(hardwareMap, follower, "turret");

        hoodController = new HoodController(hardwareMap);
        intake = new Intake(hardwareMap, "intake");
//        intakeSensor = new IntakeSensor(hardwareMap);
        intakeDistanceSensor = new IntakeDistanceSensor(hardwareMap);
        turretSensor = new TurretSensor(hardwareMap);
        gateServo = new GateServo(hardwareMap);
        liftServo = new LiftServo(hardwareMap);
        transfer = new Transfer(hardwareMap, gamepad2);


        //controllers
        //ballController = new BallController(intakeDistanceSensor, turretSensor);
        intakeController = new IntakeController(shooter, hardwareMap, "intake");
        driveController = new TeleOpDriveController(follower, gamepad1);
        lightController = new LightController(hardwareMap, intakeDistanceSensor);
        limelightController = new LimelightController(hardwareMap, "limelight", follower, telemetryM);

        //pathing
        rf12Paths = new RF12Paths(follower);
        bf12Paths = new BF12Paths(follower);
        rc12Paths = new RC12Paths(follower);
        bc12Paths = new BC12Paths(follower);
        bf6Paths = new BF6Paths(follower);
        rf6Paths = new RF6Paths(follower);


        //handlers
        shooterHandler = new ShooterHandler(telemetryM, follower, shooter, hoodController, intakeController);
        limelightHandler = new LimelightHandler(limelightController, follower);

    }

    public FSM FSM;
    public TeleOpHandler teleOpHandler;
    public void init() {
        FSM = new FSM(this);
        if (FSM != null) {
            if (isTeleop) {
                //initialize teleop only components
                teleOpHandler = new TeleOpHandler(FSM, gamepad1, gamepad2, shooterHandler, limelightController, liftServo, follower);
            }
            else if (!isTeleop){
            }
        }
    }
    Pose center = new Pose(72, 72, 0);
    public void teleStart() {
        teleOpHandler.start();
        opmodeTimer.resetTimer();
        if(PoseStorage.endPose != null){
            follower.setStartingPose(PoseStorage.endPose);
        }
        else{
            follower.setStartingPose(center);
        }
        follower.startTeleopDrive(true);
        gateServo.setPosition(0.32);
        FSM.runNew(org.firstinspires.ftc.teamcode.RIstates.management.fsm.FSM.StateName.AllianceSelect);
    }
    public void teleUpdate() {
        follower.update();
        telemetryM.update();
        teleOpHandler.update();
        driveController.update();
        turret.trackGoal(shooterHandler.alliance);
        //telemetryM.addData("intake distance", intakeDistanceSensor.test());
        //telemetryM.addData("turret distance", turretSensor.test());
        telemetryM.addData("Current state", FSM.getCurrentStateAsString());
        telemetryM.addData("Alliance", getAlliance());
        telemetryM.addData("current loc", follower.getPose());
        telemetryM.addData("flywheel vel. (right)", shooter.flyRight.getVelocity());
        telemetryM.addData("hood pos", hoodController.getPosition());
        telemetryM.addData("can shoot", shooterHandler.inRange());
        telemetryM.addData("flywheel error", shooter.getError());
        //telemetryM.addData("distance goal", shooterHandler.getTargetDistance(follower, ))
//        telemetryM.addData("full?", lightController.checkFull());
        telemetry.addData("Current state", FSM.getCurrentStateAsString());
        telemetry.addData("Alliance", getAlliance());
        //telemetry.addData("balls", ballController.getBallCount());
        telemetry.update();
        hoodController.angleHood(shooterHandler.getTargetDistance(follower, shooterHandler.alliance));
        lightController.update(
                teleOpHandler.updateLimelight,
                FSM.getCurrentStateName(),
                shooterHandler.alliance,
                shooter.averageVelocity(),
                shooter.getShooterTPS(shooterHandler.getTargetDistance(follower, shooterHandler.alliance))
        );
        if (teleOpHandler != null){
            turret.offset = teleOpHandler.offset;
        }
        if (shooterHandler != null){
            teleOpHandler.alliance = shooterHandler.alliance;
        }
//        if(!shooterHandler.shooterRunning){
//            shooter.test(600);
//        }
        //TODO: uncomment above once gate servo pos's are good

        //shooterHandler.constantShoot();
    }
    
    public void testUpdate() {
        follower.update();
        telemetryM.update();
        //teleOpHandler.update();
        //driveController.teleopNorm();
        //FSM.update();
        turret.trackGoal(shooterHandler.alliance);
        telemetryM.addData("Current state", FSM.getCurrentStateAsString());
        telemetryM.addData("Alliance", getAlliance());
        telemetryM.addData("turret pos", turret.turretPosition());
        telemetryM.addData("intake distance", intakeDistanceSensor.test());
        telemetryM.addData("ticks to move", turret.ticksToMove);
        telemetryM.addData("goal angle", turret.goalAngle);
        telemetry.addData("Current state", FSM.getCurrentStateAsString());
        telemetry.addData("Alliance", getAlliance());
        telemetry.update();
    }
    public void teleStop() {
        teleOpHandler.stop();
    }

    public void autoUpdate() {
        follower.update();
        telemetryM.update();
        PoseStorage.endPose = follower.getPose();
        hoodController.angleHood(shooterHandler.getTargetDistance(follower, shooterHandler.alliance));
        //shooterHandler.shoot();
    }

    public void setAlliance(int newAlliance) {
        shooterHandler.alliance = newAlliance;
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public String getAlliance() {
        if (shooterHandler.alliance == 1) {
            return "Blue";
        }
        else if (shooterHandler.alliance == 2) {
            return "Red";
        }
        return "unselected";
    }

}