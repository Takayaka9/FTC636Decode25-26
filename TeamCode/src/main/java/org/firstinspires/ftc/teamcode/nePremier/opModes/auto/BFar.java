package org.firstinspires.ftc.teamcode.nePremier.opModes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.nePremier.pedro.pathUpdates.F6PathUpdate;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.Alliance;
import org.firstinspires.ftc.teamcode.nePremier.utils.pedroUtils.PathUpdate;

@Autonomous
public class BFar extends OpMode {

    PathUpdate pathUpdate = null;

    @Override
    public void init() {
        pathUpdate = new F6PathUpdate(Alliance.BLUE, hardwareMap, telemetry);
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
