package org.firstinspires.ftc.teamcode.nePremier.opModes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.nePremier.pedro.pathUpdates.TestPathUpdate;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.Alliance;


/// Tester for Flee using blank pathUpdate
/// Starts at center field
/// Uses normal fleeTime
@Disabled
@Autonomous
public class FleeTester extends OpMode {
    TestPathUpdate pathUpdate = null;
    @Override
    public void init() {
        TestPathUpdate pathUpdate = new TestPathUpdate(hardwareMap, telemetry, Alliance.RED);
        pathUpdate.init();
        pathUpdate.initDependencies();
    }

    @Override
    public void loop() {
        pathUpdate.update();
    }
}
