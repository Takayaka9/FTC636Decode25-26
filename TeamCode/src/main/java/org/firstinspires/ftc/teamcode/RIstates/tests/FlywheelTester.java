package org.firstinspires.ftc.teamcode.RIstates.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RIstates.management.SystemManager;

@Configurable
@TeleOp(name = "Flywheel Tester", group = "TeleOp")
public class FlywheelTester extends OpMode {

    /// set in panels (pedro units, 12u = 1ft)
    private static double tps = 0;
    SystemManager manager;

    @Override
    public void init() {
        manager = new SystemManager(hardwareMap, telemetry, gamepad1, gamepad2, true);
        manager.init();
    }

    @Override
    public void loop() {
        manager.teleUpdate();

        manager.shooter.test(tps);

        //telemetry:
        manager.telemetryM.addData(
                "target TPS - 2800 = 6000",
                manager.shooter.getShooterTPS(manager.shooterController.getTargetDistance(manager.follower, manager.shooterController.alliance))
        );
        manager.telemetryM.addData(
                "right output power (0-1)",
                manager.shooter.outputRight
        );
        manager.telemetryM.addData(
                "left output power (0-1)",
                manager.shooter.outputLeft
        );
        manager.telemetryM.addData(
                "right velocity in tps",
                manager.shooter.flyRight.getVelocity()
        );
        manager.telemetryM.addData(
                "left velocity in tps",
                manager.shooter.flyLeft.getVelocity()
        );
    }
}
