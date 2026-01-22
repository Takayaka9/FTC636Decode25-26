package org.firstinspires.ftc.teamcode.RIstates.management;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.HoodController;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.pedro.PoseStorage;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.servo.base.GamepadServoImplEx;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.servo.GateServo;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.LimelightHandler;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Controller;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Limelight.LimelightController;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Utilities.Turret;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.color.IntakeDistanceSensor;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.ShooterHandler;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.ballController.BallController;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.servo.BeltController;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Light.LightController;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.pedro.RedPoseLib;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.pedro.RF12Paths;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.TeleOpHandler;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.color.IntakeSensor;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.color.TurretSensor;
import org.firstinspires.ftc.teamcode.RIstates.management.fsm.FSM;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Drive.Config;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Intake;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Shooter;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Drive.TeleOpDriveController;

public class SystemManager {
    public final Follower follower;
    public final TelemetryManager telemetryM;
    public final Config config;
    public final Turret turret;
    public final HoodController hoodController;
    public final Shooter shooter;
    public final Intake intake;
    public final GamepadServoImplEx gateServo;
    public final IntakeSensor intakeSensor;
    public final IntakeDistanceSensor intakeDistanceSensor;
    public final TurretSensor turretSensor;
    public final BeltController beltController;
    public final ShooterHandler shooterHandler;
    public final BallController ballController;
    public final Controller driveController;
    public final LightController lightController;
    public final LimelightController limelightController;
    public final LimelightHandler limelightHandler;
    public final PoseStorage poseStorage;

    public Gamepad gamepad1;
    public Gamepad gamepad2;


    public Timer pathTimer, actionTimer, opmodeTimer;
    public int pathState;
    public final RedPoseLib redPoseLib;

    private final Telemetry telemetry;

    private boolean isTeleop;


    public SystemManager(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, Boolean isTeleOp) {
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
        redPoseLib = new RedPoseLib();
        poseStorage = new PoseStorage();
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();

        //subsystems
        config = new Config(hardwareMap);
        shooter = new Shooter(hardwareMap, "sr", "sl");
        turret = new Turret(hardwareMap, follower, "turret");
        hoodController = new HoodController(hardwareMap);
        intake = new Intake(hardwareMap, "intake");
        intakeSensor = new IntakeSensor(hardwareMap);
        intakeDistanceSensor = new IntakeDistanceSensor(hardwareMap);
        turretSensor = new TurretSensor(hardwareMap);
        gateServo = new GateServo(hardwareMap);


        //controllers
        ballController = new BallController(intakeDistanceSensor, turretSensor);
        beltController = new BeltController(shooter, hardwareMap, "belt");
        driveController = new TeleOpDriveController(follower, gamepad1);
        lightController = new LightController(hardwareMap);
        limelightController = new LimelightController(hardwareMap, "limelight", follower, telemetryM);

        //handlers
        shooterHandler = new ShooterHandler(telemetryM, follower, shooter, hoodController, beltController, ballController);
        limelightHandler = new LimelightHandler(limelightController, follower);

    }

    public FSM FSM;
    public RF12Paths rf12Paths;
    public TeleOpHandler teleOpHandler;
    public void init() {
        FSM = new FSM(this);
        if (FSM != null) {
            if (isTeleop) {
                //initialize teleop only components
                teleOpHandler = new TeleOpHandler(FSM, gamepad1, gamepad2, shooterHandler, limelightHandler);
            }
            else if (!isTeleop){
                ///initialize paths add more paths here
                rf12Paths.buildPaths();
            }
        }
    }

    public void teleStart() {
        teleOpHandler.start();
        opmodeTimer.resetTimer();
        follower.setStartingPose(poseStorage.getPose());
        follower.startTeleopDrive();
        FSM.runNew(org.firstinspires.ftc.teamcode.RIstates.management.fsm.FSM.StateName.AllianceSelect);
    }
    public void teleUpdate() {
        follower.update();
        telemetryM.update();
        teleOpHandler.update();
        driveController.update();
        turret.trackGoal(shooterHandler.alliance);
        telemetryM.addData("intake distance", intakeSensor.test());
        telemetryM.addData("turret distance", turretSensor.test());
        telemetryM.addData("Current state", FSM.getCurrentStateAsString());
        telemetryM.addData("Alliance", getAlliance());
        telemetry.addData("Current state", FSM.getCurrentStateAsString());
        telemetry.addData("Alliance", getAlliance());
        telemetry.addData("balls", ballController.getBallCount());
        telemetry.update();
        lightController.update(teleOpHandler.updateLimelight, FSM.getCurrentStateName(), shooterHandler.alliance);
    }


    public void testUpdate() {
        follower.update();
        telemetryM.update();
        //teleOpHandler.update();
        //driveController.teleopNorm();
        //FSM.update();
        telemetryM.addData("Current state", FSM.getCurrentStateAsString());
        telemetryM.addData("Alliance", getAlliance());
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
        poseStorage.updatePose(follower.getPose());
        turret.trackGoal(shooterHandler.alliance);
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
            return "red";
        }
        else if (shooterHandler.alliance == 2) {
            return "blue";
        }
        return "unselected";
    }

}