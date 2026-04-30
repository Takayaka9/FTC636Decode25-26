package org.firstinspires.ftc.teamcode.nePremier.opModes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.nePremier.robot.commands.TransferRun;
import org.firstinspires.ftc.teamcode.nePremier.robot.systems.Transfer;
import org.firstinspires.ftc.teamcode.nePremier.utils.init.Initializer;
import org.firstinspires.ftc.teamcode.nePremier.utils.gamepadSystem.Control;
import org.firstinspires.ftc.teamcode.nePremier.utils.gamepadSystem.ControlType;
import org.firstinspires.ftc.teamcode.nePremier.utils.gamepadSystem.GamepadInput;

/**
 * Shoot Tester for testing LUT calibration, initializes the whole robot as standard.
 * Had a bit too much fun with this one, probably should have been simpler
 */
@TeleOp
public class PatrickShootTest extends OpMode {
    Initializer i = null;
    Control opModeShoot = null;
    Transfer transfer;
    TransferRun tRunn;
    Control runt;
    enum action {
        op,
        test,
        off
    }
    @Configurable
    public static class PatrickShootTestParams {
        static action currentAction = action.off;
        static int testSpeed = 500;
    }

    @Override
    public void init() {
        i = new Initializer(gamepad1, gamepad2, hardwareMap, telemetry);
        opModeShoot = new Control(ControlType.Auto, i.constantFlywheelSpin);
        transfer = new Transfer(hardwareMap);
        tRunn = new TransferRun(transfer);
        runt = new Control(GamepadInput.right_bumper, gamepad1, ControlType.Hold, tRunn);
    }

    @Override
    public void loop() {
        i.follower.update();
        opModeShoot.update();
        runt.update();
        switch (PatrickShootTestParams.currentAction) {
            case op:
                opModeShoot.run();
                break;
            case test:
                opModeShoot.stop();
                i.shooter.test(PatrickShootTestParams.testSpeed);
                break;
            case off:
                opModeShoot.stop();
                i.shooter.stop();
                break;
        }
        FlywheelTelemetryHelper.loop(i);
    }
}
