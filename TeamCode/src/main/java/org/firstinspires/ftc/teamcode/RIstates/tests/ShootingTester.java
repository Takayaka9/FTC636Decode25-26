package org.firstinspires.ftc.teamcode.RIstates.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RIstates.management.SystemManager;

public class ShootingTester extends OpMode {
    SystemManager manager;

    @Override
    public void init() {
        manager.Init();
    }
    @Override
    public void loop() {
        manager = new SystemManager(hardwareMap, gamepad1, gamepad2, true);
        manager.teleUpdate();
        manager.shooterController.shoot();
        manager.telemetryM.addData(
                "target distance",
                manager.shooterController.getTargetDistance(manager.follower, manager.shooterController.alliance)
        );
    }

}
