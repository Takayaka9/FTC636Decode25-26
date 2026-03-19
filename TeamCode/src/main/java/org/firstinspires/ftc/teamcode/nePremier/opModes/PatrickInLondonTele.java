package org.firstinspires.ftc.teamcode.nePremier.opModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.nePremier.tele.TeleControls;

@TeleOp
public class PatrickInLondonTele extends OpMode {
    public TeleControls teleControls = null;
    @Override
    public void init() {
        teleControls = new TeleControls(gamepad1, gamepad1,hardwareMap, telemetry);
    }

    @Override
    public void start() {
        teleControls.start();
    }

    @Override
    public void loop() {
        teleControls.update();
    }
}
