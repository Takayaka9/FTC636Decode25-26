package org.firstinspires.ftc.teamcode.RIstates.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RIstates.management.SystemManager;

@Configurable
@TeleOp(name = "Flywheel Tester", group = "TeleOp")
public class FlywheelTester extends OpMode {

    /// set in panels (pedro units, 12u = 1ft)
    private static double tps = 50;
    SystemManager manager;

    @Override
    public void init() {
        manager = new SystemManager(hardwareMap, telemetry, gamepad1, gamepad2, true, true);
        manager.init();
    }

    @Override
    public void loop() {
        manager.testUpdate();

        manager.shooter.test(tps);

        //telemetry:
        manager.telemetryM.addData(
                "target TPS - 2800 = 6000",
                tps
        );
        manager.telemetryM.addData(
                "calc TPS",
                manager.shooter.getShooterTPS(manager.shooterHandler.getTargetDistance(manager.follower, manager.shooterHandler.alliance))
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
        manager.telemetryM.addData(
                "goal distance red", manager.shooterHandler.getTargetDistance(manager.follower, 2)
        );
        manager.telemetryM.addData(
                "goal distance blue", manager.shooterHandler.getTargetDistance(manager.follower, 1)
        );
    }
}
