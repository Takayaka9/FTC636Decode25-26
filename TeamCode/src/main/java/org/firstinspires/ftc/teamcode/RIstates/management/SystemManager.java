package org.firstinspires.ftc.teamcode.RIstates.management;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.states.controllers.PathingController;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.states.controllers.pedro.PoseLib;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.states.controllers.pedro.RF12Paths;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.TeleOpHandler;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.FSM;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.states.controllers.subsystems.Config;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.states.controllers.subsystems.Hood;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.states.controllers.subsystems.Intake;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.states.controllers.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.states.controllers.ShooterController;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.states.controllers.TeleOpDriveController;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.states.controllers.subsystems.Turret;

public class SystemManager {
    public final Follower follower;
    public final TelemetryManager telemetryM;
    public final Config config;
    public final Turret turret;
    public final Hood hood;
    public final Shooter shooter;
    public final Intake intake;
    public final ShooterController shooterController;
    public final TeleOpDriveController driveController;


    public Gamepad gamepad1;
    public Gamepad gamepad2;


    public Timer pathTimer, actionTimer, opmodeTimer;
    public int pathState;
    public final PoseLib poseLib;

    private boolean isTeleop;


    public SystemManager(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Boolean isTeleOp) {
        ///DO NOT CHANGE THE INIT ORDER, add new stuff in it's respective place to avoid NullPointer

        //dependencies
        isTeleop = isTeleOp;
        pathState = 0;
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);

        //auto dependencies
        poseLib = new PoseLib();
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();

        //subsystems
        config = new Config(hardwareMap);
        turret = new Turret(hardwareMap, "turret");
        shooter = new Shooter(hardwareMap, "flyRight", "flyLeft");
        hood = new Hood(hardwareMap, "servo");
        intake = new Intake(hardwareMap, "intake");

        //controllers
        driveController = new TeleOpDriveController(hardwareMap, follower, gamepad1);
        shooterController = new ShooterController(telemetryM, follower, shooter, hood, turret);
    }

    public FSM FSM;
    public RF12Paths rf12Paths;
    public TeleOpHandler teleOpHandler;
    public void init() {
        FSM = new FSM(this);
        if (FSM != null) {
            if (isTeleop) {
                //initialize teleop only components
                teleOpHandler = new TeleOpHandler(FSM, gamepad1, gamepad2);
            }
            else if (!isTeleop){
                //initialize paths
                rf12Paths.buildPaths();
            }
        }
    }

    public void teleUpdate() {
        follower.update();
        telemetryM.update();
        teleOpHandler.update();
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
}