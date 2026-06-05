package org.firstinspires.ftc.teamcode.nePremier.opModes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.nePremier.pedro.pathUpdates.FarUpdate;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.Alliance;
import org.firstinspires.ftc.teamcode.nePremier.utils.pedroUtils.PathUpdate;
@Disabled
@Autonomous
public class RFar extends OpMode {

    PathUpdate pathUpdate = null;

    @Override
    public void init() {
        pathUpdate = new FarUpdate(Alliance.RED, hardwareMap, telemetry);
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
