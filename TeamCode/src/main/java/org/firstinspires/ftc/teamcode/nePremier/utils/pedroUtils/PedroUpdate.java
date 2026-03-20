package org.firstinspires.ftc.teamcode.nePremier.utils.pedroUtils;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.nePremier.pedro.autoConstants.AutoConstants;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.Alliance;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.CurrentAlliance;
import org.firstinspires.ftc.teamcode.nePremier.utils.init.Initializer;
import org.firstinspires.ftc.teamcode.nePremier.utils.inputSystem.Control;
import org.firstinspires.ftc.teamcode.nePremier.utils.inputSystem.ControlType;
import org.firstinspires.ftc.teamcode.nePremier.utils.timers.GenericTime;
import org.firstinspires.ftc.teamcode.nePremier.utils.timers.SolversTiming;

abstract class PedroUpdate extends Initializer {
    public final Control intakeControl;
    public final Control outtakeControl;
    public final Control shootControl;
    public final Control constantControls;
    private final Control zeroTurretControl;
    public final ElapsedTime opModeTimer = new ElapsedTime();
    public final GenericTime shootTimer = new SolversTiming();
    public final GenericTime fleeTimer = new SolversTiming();
    public boolean fled;
    public int pathState;


    public PedroUpdate(HardwareMap hardwareMap, Telemetry telemetry, Alliance alliance) {
        super(null, null, hardwareMap, telemetry);
        CurrentAlliance.alliance = alliance;
        intakeControl = new Control(ControlType.Auto, transferRun);
        outtakeControl = new Control(ControlType.Auto, outake);
        shootControl = new Control(ControlType.Auto, shoot);
        constantControls = new Control(ControlType.Auto, turretHoodUpdate, constantFlywheelSpin, draw);
        zeroTurretControl = new Control(ControlType.Auto, resetForTele);
        shootTimer.create();
        shootTimer.setLength(AutoConstants.shootTime);
        fleeTimer.create();
        fleeTimer.setLength(AutoConstants.fleeTime);
        pathState = 0;
        fled = false;
    }

    public final boolean atPose(Pose pose) {
        return follower.atPose(pose, AutoConstants.globalPoseTolerance, AutoConstants.globalPoseTolerance);
    }

    public final void zeroTurret() {
        constantControls.stop();
        shootControl.stop();
        zeroTurretControl.run();
    }

    public final void initDependencies() {
        opModeTimer.reset();
        pathState = 0;
        fled = false;
    }

    public final void startDependencies() {
        shootTimer.setLength(AutoConstants.shootTime);
        fleeTimer.setLength(AutoConstants.fleeTime);
        opModeTimer.reset();
        pathState = 0;
        fled = false;
        constantControls.run();
    }

    public final void updateDependencies() {
        intakeControl.update();
        outtakeControl.update();
        shootControl.update();
        constantControls.update();
        telemetryM.update();
        follower.update();
        telemetryM.addData("Path State", pathState);
    }

}
