package org.firstinspires.ftc.teamcode.NewEnglands.utils.pedroUtils;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.alliance.Alliance;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.alliance.CurrentAlliance;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.init.Initializer;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.inputSystem.Control;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.inputSystem.ControlType;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.inputSystem.GamepadInput;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.pedroUtils.interfaceUtils.PathUpdate;

public class BasePedroUpdate extends Initializer implements PathUpdate {
    private final Control intakeRB;
    private final Control outtakeLB;
    private final Control shootA;
    private final Control liftY;
    private final Control constants;

    public BasePedroUpdate(Gamepad gamepad1, Gamepad gamepad2, HardwareMap hardwareMap, Telemetry telemetry, Alliance alliance) {
        super(gamepad1, gamepad2, hardwareMap, telemetry);
        Alliance currentAlliance = alliance;
        intakeRB = new Control(ControlType.Hold, intake);
        outtakeLB = new Control(ControlType.Hold, outake);
        shootA = new Control(ControlType.Hold, shoot);
        liftY = new Control(ControlType.Toggle, liftBot);
        constants = new Control(ControlType.Continuous, turretHoodUpdate, shoot, makeMoves);
    }



    public void init() {}


    public void update() {
        intakeRB.update();
        outtakeLB.update();
        shootA.update();
        liftY.update();
        constants.update();
    }

    public void stop() {}

}
