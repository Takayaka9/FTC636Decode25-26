package org.firstinspires.ftc.teamcode.nePremier.tele;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.nePremier.utils.init.Initializer;
import org.firstinspires.ftc.teamcode.nePremier.utils.inputSystem.Control;
import org.firstinspires.ftc.teamcode.nePremier.utils.inputSystem.ControlType;
import org.firstinspires.ftc.teamcode.nePremier.utils.inputSystem.GamepadInput;
import org.firstinspires.ftc.teamcode.nePremier.utils.pedroUtils.Drawing;

public class TeleControls extends Initializer {
    private final Control allianceBlue;
    private final Control allianceRed;
    private final Control intake;
    private final Control outtake;
    private final Control shoot;
    private final Control weFucked;
    private final Control liftUp;
    private final Control constantControls;
    private final Control down;

    /// THIS CONSTRUCTOR SERVES AS OUR INIT METHOD
    public TeleControls(Gamepad gamepad1, Gamepad gamepad2, HardwareMap hardwareMap, Telemetry telemetry) {
        super(gamepad1, gamepad2, hardwareMap, telemetry);
        Drawing.init();
        allianceBlue = new Control(GamepadInput.x, gamepad1, ControlType.Hold, blue);
        allianceRed = new Control(GamepadInput.b, gamepad1, ControlType.Hold, red);
        intake = new Control(GamepadInput.right_bumper, gamepad2, ControlType.Hold, transferRun);
        outtake = new Control(GamepadInput.left_bumper, gamepad2, ControlType.Hold, outake);
        shoot = new Control(GamepadInput.a, gamepad2, ControlType.Hold, shootCommand);
        weFucked = new Control(GamepadInput.x, gamepad2, ControlType.Toggle, ohNoWeFucked);
        liftUp = new Control(GamepadInput.a, gamepad1, ControlType.Hold, liftBot);
        down = new Control(GamepadInput.y, gamepad1, ControlType.Hold, liftDown);
        constantControls = new Control(ControlType.Auto, turretHoodUpdate, makeMoves, constantFlywheelSpin, draw
//              , localizer
                );
    }

//    public void init_loop() {}

    public void start() {
        follower.update();
        constantControls.run();
    }

    public void update() {
        allianceBlue.update();
        allianceRed.update();
        intake.update();
        outtake.update();
        shoot.update();
        weFucked.update();
        liftUp.update();
        down.update();
        constantControls.update();
        follower.update();
        telemetryM.update();
    }

    public void stop() {
        constantControls.stop();
    }

}
