package org.firstinspires.ftc.teamcode.RIstates.management;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Controller;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Limelight.Limelight;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Limelight.LimelightController;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.ShooterHandler;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.ballController.BallController;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Belt.BeltController;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Light.LightController;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.pedro.PoseLib;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.pedro.RF12Paths;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.TeleOpHandler;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.color.IntakeSensor;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.color.TurretSensor;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.RIstates.management.fsm.FSM;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Drive.Config;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Hood;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Intake;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Shooter;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Drive.TeleOpDriveController;
//import org.firstinspires.ftc.teamcode.RIstates.management.handlers.fsm.states.controllers.subsystems.Turret;

public class SystemManager {
    public final Follower follower;
    public final TelemetryManager telemetryM;
    public final Config config;
    //public final Turret turret;
    public final Hood hood;
    public final Shooter shooter;
    public final Intake intake;
    public final Limelight limelight;
    public final IntakeSensor intakeSensor;
    public final TurretSensor turretSensor;
    public final BeltController beltController;
    public final ShooterHandler shooterHandler;
    public final BallController ballController;
    public final Controller driveController;
    public final LightController lightController;
    //public final LimelightController limelightController;

    public Gamepad gamepad1;
    public Gamepad gamepad2;


    public Timer pathTimer, actionTimer, opmodeTimer;
    public int pathState;
    public final PoseLib poseLib;

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
        poseLib = new PoseLib();
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();

        //subsystems
        config = new Config(hardwareMap);
        //turret = new Turret(hardwareMap, "turret");
        shooter = new Shooter(hardwareMap, "sr", "sl");
        hood = new Hood(hardwareMap, "hood");
        intake = new Intake(hardwareMap, "intake");
        intakeSensor = new IntakeSensor(hardwareMap, telemetryM);
        turretSensor = new TurretSensor(hardwareMap, telemetryM);
        //limelight = new Limelight(hardwareMap, "limelight");


        //controllers
        ballController = new BallController(intakeSensor, turretSensor);
        beltController = new BeltController(shooter, hardwareMap, "belt");
        driveController = new TeleOpDriveController(follower, gamepad1);
        lightController = new LightController(hardwareMap);
        //limelightController = new LimelightController(limelight)

        //handlers
        shooterHandler = new ShooterHandler(telemetryM, follower, shooter, hood, beltController, lightController, ballController);


    }

    public FSM FSM;
    public RF12Paths rf12Paths;
    public TeleOpHandler teleOpHandler;
    public void init() {
        FSM = new FSM(this);
        if (FSM != null) {
            if (isTeleop) {
                //initialize teleop only components
                teleOpHandler = new TeleOpHandler(FSM, gamepad1, gamepad2, shooterHandler);
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
        follower.startTeleopDrive();
        FSM.runNew(org.firstinspires.ftc.teamcode.RIstates.management.fsm.FSM.StateName.AllianceSelect);
    }
    public void teleUpdate() {
        follower.update();
        telemetryM.update();
        teleOpHandler.update();
        driveController.update();
        FSM.update();
        telemetryM.addData("Current state", FSM.getCurrentState());
        telemetryM.addData("Alliance", getAlliance());
        telemetry.addData("Current state", FSM.getCurrentState());
        telemetry.addData("Alliance", getAlliance());
        telemetry.addData("balls", ballController.getBallCount());
        telemetry.update();
    }


    public void testUpdate() {
        follower.update();
        telemetryM.update();
        //teleOpHandler.update();
        //driveController.teleopNorm();
        //FSM.update();
        telemetryM.addData("Current state", FSM.getCurrentState());
        telemetryM.addData("Alliance", getAlliance());
        telemetry.addData("Current state", FSM.getCurrentState());
        telemetry.addData("Alliance", getAlliance());
        telemetry.update();
    }
    public void teleStop() {
        teleOpHandler.stop();
    }

    public void autoUpdate() {
        follower.update();
        telemetryM.update();
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