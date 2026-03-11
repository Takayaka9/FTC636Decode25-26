package org.firstinspires.ftc.teamcode.NewEnglands.utils.pedroUtils;

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
    public final Control intakeRB;
    public final Control outtakeLB;
    public final Control shootA;
    public final Control liftY;
    public final Control constants;
    public final ElapsedTime opModeTimer = new ElapsedTime();
    public final GenericTime shootTimer = new SolversTiming();
    public final GenericTime fleeTimer = new SolversTiming();
    public boolean fled = false;
    public int pathState = 0;


    public PedroUpdate(HardwareMap hardwareMap, Telemetry telemetry, Alliance alliance) {
        super(null, null, hardwareMap, telemetry);
        CurrentAlliance.alliance = alliance;
        intakeRB = new Control(ControlType.Hold, intake);
        outtakeLB = new Control(ControlType.Hold, outake);
        shootA = new Control(ControlType.Hold, shoot);
        liftY = new Control(ControlType.Toggle, liftBot);
        constants = new Control(ControlType.Continuous, turretHoodUpdate, shoot, makeMoves);
        shootTimer.create();
        shootTimer.setLength(AutoConstants.shootTime);
        fleeTimer.create();
        fleeTimer.setLength(AutoConstants.fleeTime);
        pathState = 0;
        fled = false;
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
        intakeRB.update();
        outtakeLB.update();
        shootA.update();
        liftY.update();
        constants.update();
        telemetryM.update();
        follower.update();
        Drawing.drawDebug(follower);
        telemetryM.addData("Path State", pathState);
    }


}
