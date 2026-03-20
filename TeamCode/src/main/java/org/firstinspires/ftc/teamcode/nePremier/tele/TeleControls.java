package org.firstinspires.ftc.teamcode.nePremier.tele;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.nePremier.utils.init.Initializer;
import org.firstinspires.ftc.teamcode.nePremier.utils.inputSystem.Control;
import org.firstinspires.ftc.teamcode.nePremier.utils.inputSystem.ControlType;
import org.firstinspires.ftc.teamcode.nePremier.utils.inputSystem.GamepadInput;

public class TeleControls extends Initializer {
    private final Control allianceBlueX;
    private final Control allianceRedB;
    private final Control intakeRB;
    private final Control outtakeLB;
    private final Control shootA;
    private final Control liftB;
    private final Control constantControls;
    private final Control downY;

    public TeleControls(Gamepad gamepad1, Gamepad gamepad2, HardwareMap hardwareMap, Telemetry telemetry) {
        super(gamepad1, gamepad2, hardwareMap, telemetry);
        allianceBlueX = new Control(GamepadInput.x, gamepad1, ControlType.Hold, blue);
        allianceRedB = new Control(GamepadInput.b, gamepad1, ControlType.Hold, red);
        intakeRB = new Control(GamepadInput.right_bumper, gamepad2, ControlType.Hold, transferRun);
        outtakeLB = new Control(GamepadInput.left_bumper, gamepad2, ControlType.Hold, outake);
        shootA = new Control(GamepadInput.a, gamepad2, ControlType.Hold, shoot);
        liftB = new Control(GamepadInput.b, gamepad2, ControlType.Hold, liftBot);
        downY = new Control(GamepadInput.y, gamepad2, ControlType.Hold, liftDown);
        constantControls = new Control(ControlType.Auto, turretHoodUpdate, makeMoves, constantFlywheelSpin, draw, localizer);
    }


//    public void init() {}

//    public void init_loop() {}

    public void start() {
        constantControls.run();
    }

    public void update() {
        allianceBlueX.update();
        allianceRedB.update();
        intakeRB.update();
        outtakeLB.update();
        shootA.update();
        liftB.update();
        downY.update();
        constantControls.update();
    }

//    public void stop() {}

}
