package org.firstinspires.ftc.teamcode.RIstates.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RIstates.management.SystemManager;

@Configurable
@TeleOp(name = "States TeleOp 3", group = "TeleOp")
public class ShootingTester extends OpMode {

    //    Subsystems + Follower
    SystemManager manager;
    HardwareMap hardwareMap;


    @Override
    public void loop() {
        manager.teleUpdate();

        //shooterController data
        manager.telemetryM.addData(
                "shooter running - boolean",
                manager.shooterController.shooterRunning
        );
        manager.telemetryM.addData(
                "target distance - 1ft = 12u (pedro pose unit)",
                manager.shooterController.getTargetDistance(manager.follower, manager.shooterController.alliance)
        );
        manager.telemetryM.addData(
                "alliance - 0=init 1=red 2=blue",
                manager.shooterController.alliance
        );

        //shooter data
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

        //TODO: turret data




    }

    @Override
    public void init() {
        manager = new SystemManager(hardwareMap, telemetry, gamepad1, gamepad2, true);
        manager.init();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        manager.teleStart();
    }

    @Override
    public void stop() {
        manager.teleStop();
    }

}
