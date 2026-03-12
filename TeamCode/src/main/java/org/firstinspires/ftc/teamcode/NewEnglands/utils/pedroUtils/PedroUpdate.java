package org.firstinspires.ftc.teamcode.NewEnglands.utils.pedroUtils;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.NewEnglands.pedro.AutoConstants;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.alliance.Alliance;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.alliance.CurrentAlliance;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.init.Initializer;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.inputSystem.Control;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.inputSystem.ControlType;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.timers.GenericTime;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.timers.SolversTiming;

abstract class PedroUpdate extends Initializer {
    public final Control intakeControl;
    public final Control outtakeControl;
    public final Control shootControl;
    public final Control liftControl;
    public final Control constantControls;
    public final ElapsedTime opModeTimer = new ElapsedTime();
    public final GenericTime shootTimer = new SolversTiming();
    public final GenericTime fleeTimer = new SolversTiming();
    public boolean fled = false;
    public int pathState = 0;


    public PedroUpdate(HardwareMap hardwareMap, Telemetry telemetry, Alliance alliance) {
        super(null, null, hardwareMap, telemetry);
        CurrentAlliance.alliance = alliance;
        intakeControl = new Control(ControlType.Hold, intake);
        outtakeControl = new Control(ControlType.Hold, outake);
        shootControl = new Control(ControlType.Hold, shoot);
        liftControl = new Control(ControlType.Toggle, liftBot);
        constantControls = new Control(ControlType.Continuous, turretHoodUpdate, shoot, makeMoves);
        shootTimer.create();
        shootTimer.setLength(AutoConstants.shootTime);
        fleeTimer.create();
        fleeTimer.setLength(AutoConstants.fleeTime);
        pathState = 0;
        fled = false;
    }

    public final boolean atPose(Pose pose) {
        if (follower.atPose(pose, AutoConstants.poseTolerance, AutoConstants.poseTolerance)) {
            return true;
        } else {
            return false;
        }
    }

    public final void initDependencies() {
        opModeTimer.reset();
        Drawing.init();
        pathState = 0;
        fled = false;
    }

    public final void startDependencies() {
        shootTimer.setLength(AutoConstants.shootTime);
        fleeTimer.setLength(AutoConstants.fleeTime);
        opModeTimer.reset();
        pathState = 0;
        fled = false;
    }


    public final void updateDependencies() {
        intakeControl.update();
        outtakeControl.update();
        shootControl.update();
        liftControl.update();
        constantControls.update();
        telemetryM.update();
        follower.update();
        Drawing.drawDebug(follower);
        telemetryM.addData("Path State", pathState);
    }


}
