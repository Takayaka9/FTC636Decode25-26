package org.firstinspires.ftc.teamcode.nePremier.opModes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.nePremier.pedro.autoConstants.AutoConstants;
import org.firstinspires.ftc.teamcode.nePremier.pedro.pathUpdates.EmadAllianceUpdate;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.Alliance;
import org.firstinspires.ftc.teamcode.nePremier.utils.pedroUtils.PathUpdate;

@Autonomous
public class RCurvyAlliance extends OpMode {

    PathUpdate pathUpdate = null;

    @Override
    public void init() {
        AutoConstants.gateCycles = 2;
        pathUpdate = new EmadAllianceUpdate(Alliance.RED, hardwareMap, telemetry);
        pathUpdate.init();
    }

    @Override
    public void start() {
        pathUpdate.start();
    }

    @Override
    public void loop() {
        pathUpdate.update();
    }

}
