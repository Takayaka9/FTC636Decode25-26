package org.firstinspires.ftc.teamcode.RIstates.tests;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RIstates.management.SystemManager;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.states.controllers.subsystems.Hood;
import org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM.states.controllers.subsystems.Shooter;

public class ShootingTester extends OpMode {
    SystemManager manager;

    @Override
    public void init() {
    }
    @Override
    public void loop() {
        manager = new SystemManager(hardwareMap, gamepad1, gamepad2, true);
        manager.teleUpdate();
        manager.shooterController.shoot();
        manager.telemetryM.addData(
                "target distance",
                manager.shooterController.getTargetDistance(manager.follower, manager.alliance)
        );
    }

}
