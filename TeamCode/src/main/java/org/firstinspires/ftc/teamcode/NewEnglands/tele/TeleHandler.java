package org.firstinspires.ftc.teamcode.NewEnglands.tele;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.alliance.Alliance;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.init.Initializer;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.inputSystem.Control;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.inputSystem.ControlType;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.inputSystem.GamepadInput;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.pedroUtils.interfaceUtils.PathUpdate;

public class TeleHandler extends Initializer {
    private final Control allianceBlueX;
    private final Control allianceRedB;
    private final Control intakeRB;
    private final Control outtakeLB;
    private final Control shootA;
    private final Control liftY;

    public TeleHandler(Gamepad gamepad1, Gamepad gamepad2, HardwareMap hardwareMap, Telemetry telemetry) {
        super(gamepad1, gamepad2, hardwareMap, telemetry);
        allianceBlueX = new Control(GamepadInput.x, gamepad1, ControlType.Hold, commandLoop);
        allianceRedB = new Control(GamepadInput.b, gamepad1, ControlType.Hold, commandLoop);
        intakeRB = new Control(GamepadInput.right_bumper, gamepad1, ControlType.Hold, commandLoop);
        outtakeLB = new Control(GamepadInput.left_bumper, gamepad1, ControlType.Hold, commandLoop);
        shootA = new Control(GamepadInput.a, gamepad1, ControlType.Hold, commandLoop);
        liftY = new Control(GamepadInput.y, gamepad1, ControlType.Toggle, commandLoop);
    }
    
    private void updateControls() {
        allianceBlueX.update();
        allianceRedB.update();
        intakeRB.update();
        outtakeLB.update();
        shootA.update();
        liftY.update();
    }

    public void start() {
        commandLoop.runCommand(turretHoodUpdate);
        commandLoop.runCommand(shoot);
        commandLoop.runCommand(makeMoves);
    }

//    public void init_loop() {}

//    public void start() {}

    public void update() {
        updateControls();
        commandLoop.loop();
    }

//    public void stop() {}

}
