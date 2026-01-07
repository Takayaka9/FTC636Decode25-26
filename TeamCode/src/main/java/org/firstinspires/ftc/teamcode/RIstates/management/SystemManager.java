package org.firstinspires.ftc.teamcode.RIstates.management;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.fsm.states.controllers.BeltController;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.fsm.states.controllers.pedro.PoseLib;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.fsm.states.controllers.pedro.RF12Paths;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.TeleOpHandler;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.fsm.states.controllers.subsystems.Belt;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.fsm.FSM;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.fsm.states.controllers.subsystems.Config;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.fsm.states.controllers.subsystems.Hood;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.fsm.states.controllers.subsystems.Intake;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.fsm.states.controllers.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.fsm.states.controllers.ShooterController;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.fsm.states.controllers.TeleOpDriveController;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.fsm.states.controllers.subsystems.Turret;

public class SystemManager {
    public final Follower follower;
    public final TelemetryManager telemetryM;
    public final Config config;
    public final Turret turret;
    public final Hood hood;
    public final Shooter shooter;
    public final Intake intake;
    public final BeltController beltController;
    public final ShooterController shooterController;
    public final TeleOpDriveController driveController;


    public final Belt belt;

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
        belt = new Belt(hardwareMap, "belt");
        turret = new Turret(hardwareMap, "turret");
        shooter = new Shooter(hardwareMap, "sr", "sl");
        hood = new Hood(hardwareMap, "hood");
        intake = new Intake(hardwareMap, "intake");

        //controllers
        beltController = new BeltController(belt, shooter);
        shooterController = new ShooterController(telemetryM, follower, shooter, hood, turret, beltController);
        driveController = new TeleOpDriveController(follower, gamepad1);
    }

    public FSM FSM;
    public RF12Paths rf12Paths;
    public TeleOpHandler teleOpHandler;
    public void init() {
        FSM = new FSM(this);
        if (FSM != null) {
            if (isTeleop) {
                //initialize teleop only components
                teleOpHandler = new TeleOpHandler(FSM, gamepad1, gamepad2, shooterController);
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
        FSM.runNew(org.firstinspires.ftc.teamcode.RIstates.management.handlers.fsm.FSM.StateName.AllianceSelect);
    }
    public void teleUpdate() {
        follower.update();
        telemetryM.update();
        teleOpHandler.update();
        driveController.teleopNorm();
        FSM.update();
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
        shooterController.alliance = newAlliance;
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public String getAlliance() {
        if (shooterController.alliance == 1) {
            return "red";
        }
        else if (shooterController.alliance == 2) {
            return "blue";
        }
        return "unselected";
    }

}