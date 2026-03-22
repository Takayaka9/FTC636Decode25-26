package org.firstinspires.ftc.teamcode.nePremier.opModes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.CurrentAlliance;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.TDistHelper;
import org.firstinspires.ftc.teamcode.nePremier.utils.init.Initializer;
import org.firstinspires.ftc.teamcode.nePremier.utils.inputSystem.Control;
import org.firstinspires.ftc.teamcode.nePremier.utils.inputSystem.ControlType;

/**
 * Shoot Tester for testing LUT calibration, initializes the whole robot as standard.
 * Had a bit too much fun with this one, probably should have been simpler
 */
@TeleOp
public class PatrickShootTest extends OpMode {
    Initializer i = null;
    Control opModeShoot = null;
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
    }

    @Override
    public void loop() {
        i.telemetryM.update();
        i.follower.update();
        opModeShoot.update();
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
//        FlywheelTelemetryHelper.loop(i);
        i.telemetryM.addData(
                "target distance - 1ft = 12u (pedro pose unit)",
                TDistHelper.getTargetDistance(i.follower.getPose(), CurrentAlliance.alliance)
        );
        i.telemetryM.addData(
                "alliance",
                CurrentAlliance.alliance
        );

        //shooter data
        i.telemetryM.addData(
                "target TPS - 2800 = 6000",
                i.shooter.getShooterTPS(TDistHelper.getTargetDistance(i.follower.getPose(), CurrentAlliance.alliance))
        );
        i.telemetryM.addData(
                "1 output power (0-1)",
                i.shooter.get1Power()
        );
        i.telemetryM.addData(
                "2 output power (0-1)",
                i.shooter.get2Power()
        );
        i.telemetryM.addData(
                "2 velocity in tps",
                i.shooter.getShooter2tps()
        );
        i.telemetryM.addData(
                "1 velocity in tps",
                i.shooter.getShooter1tps()
        );
        i.telemetryM.addData(
                "average velocity in tps",
                i.shooter.getAverageVelocity()
        );
    }
}
