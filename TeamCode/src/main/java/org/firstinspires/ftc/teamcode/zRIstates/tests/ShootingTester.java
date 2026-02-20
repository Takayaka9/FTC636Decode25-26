package org.firstinspires.ftc.teamcode.zRIstates.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.zRIstates.management.SystemManager;

@Disabled
@Configurable
@TeleOp(name = "Shooting Tester", group = "TeleOp")
public class ShootingTester extends OpMode {

    //    Subsystems + Follower
    SystemManager manager;

    @Override
    public void loop() {
        manager.testUpdate();

        //shooterHandler data
        manager.telemetryM.addData(
                "shooter running - boolean",
                manager.shooterHandler.shooterRunning
        );
        manager.telemetryM.addData(
                "target distance - 1ft = 12u (pedro pose unit)",
                manager.shooterHandler.getTargetDistance(manager.follower, manager.shooterHandler.alliance)
        );
        manager.telemetryM.addData(
                "alliance - 0=init 1=red 2=blue",
                manager.shooterHandler.alliance
        );

        //shooter data
        manager.telemetryM.addData(
                "target TPS - 2800 = 6000",
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

        //TODO: turret data

        //HoodController data
        manager.telemetryM.addData("angle", manager.hoodController.angle);


    }

    @Override
    public void init() {
        manager = new SystemManager(hardwareMap, telemetry, gamepad1, gamepad2, true, true);
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
