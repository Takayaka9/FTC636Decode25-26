package org.firstinspires.ftc.teamcode.RIstates.management;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

    public final FSM FSM;

    public final TeleOpHandler teleOpHandler;

    public Gamepad gamepad1;
    public Gamepad gamepad2;






    public SystemManager(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Boolean isTeleOp) {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        config = new Config(hardwareMap);
        turret = new Turret(hardwareMap, "turret");
        shooter = new Shooter(hardwareMap, "flyRight", "flyLeft");
        hood = new Hood(hardwareMap, "servo");
        intake = new Intake(hardwareMap, "intake");
        shooterController = new ShooterController(shooter, hood, turret, follower);

        //TODO: potentially only init in teleop?
            this.gamepad1 = gamepad1;
            this.gamepad2 = gamepad2;
            driveController = new TeleOpDriveController(hardwareMap, follower, gamepad1);
            FSM = new FSM(SystemManager.this);
            teleOpHandler = new TeleOpHandler(FSM, gamepad1, gamepad2);
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


    public int alliance = 0;
    public void setAlliance(int newAlliance) {
        alliance = newAlliance;
    }
}