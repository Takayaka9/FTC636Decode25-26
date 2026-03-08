package org.firstinspires.ftc.teamcode.NewEnglands.tele;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.init.Initializer;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.inputSystem.Control;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.inputSystem.ControlType;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.inputSystem.GamepadInput;

public class TeleHandler extends Initializer {
    private final Control allianceBlueX;
    private final Control allianceRedB;
    private final Control intakeRB;
    private final Control outtakeLB;
    private final Control shootA;
    private final Control liftY;
    private final Control constants;

    public TeleHandler(Gamepad gamepad1, Gamepad gamepad2, HardwareMap hardwareMap, Telemetry telemetry) {
        super(gamepad1, gamepad2, hardwareMap, telemetry);
        allianceBlueX = new Control(GamepadInput.x, gamepad1, ControlType.Hold, blue);
        allianceRedB = new Control(GamepadInput.b, gamepad1, ControlType.Hold, red);
        intakeRB = new Control(GamepadInput.right_bumper, gamepad1, ControlType.Hold, intake);
        outtakeLB = new Control(GamepadInput.left_bumper, gamepad1, ControlType.Hold, outake);
        shootA = new Control(GamepadInput.a, gamepad1, ControlType.Hold, shoot);
        liftY = new Control(GamepadInput.y, gamepad1, ControlType.Toggle, liftBot);
        constants = new Control(ControlType.Continuous, turretHoodUpdate, shoot, makeMoves);
    }


    public void init() {}

//    public void init_loop() {}

//    public void start() {}

    public void update() {
        allianceBlueX.update();
        allianceRedB.update();
        intakeRB.update();
        outtakeLB.update();
        shootA.update();
        liftY.update();
        constants.update();
    }

//    public void stop() {}

}
