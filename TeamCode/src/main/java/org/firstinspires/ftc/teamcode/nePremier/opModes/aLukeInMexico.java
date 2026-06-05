package org.firstinspires.ftc.teamcode.nePremier.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.nePremier.tele.TeleControls;
import org.firstinspires.ftc.teamcode.nePremier.tele.TestControls;
@Disabled

@TeleOp()
public class aLukeInMexico extends OpMode {
    public TestControls teleControls = null;
    @Override
    public void init() {
        teleControls = new TestControls(gamepad1, gamepad2, hardwareMap, telemetry);
    }

    @Override
    public void start() {
        teleControls.start();
    }

    @Override
    public void loop() {
        teleControls.update();
    }

    @Override
    public void stop() {
        teleControls.stop();
    }
}
