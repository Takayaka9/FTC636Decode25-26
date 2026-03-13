package org.firstinspires.ftc.teamcode.NewEnglands.opModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.NewEnglands.pedro.pathUpdates.C9PathUpdate;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.alliance.Alliance;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.pedroUtils.PathUpdate;

@Autonomous
public class BC9v0 extends OpMode {

    PathUpdate pathUpdate;

    @Override
    public void init() {
        pathUpdate = new C9PathUpdate(Alliance.BLUE, hardwareMap, telemetry);
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
