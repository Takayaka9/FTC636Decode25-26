package org.firstinspires.ftc.teamcode.NewEnglands.opModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.NewEnglands.tele.TeleHandler;

@TeleOp
public class PatrickInLondonTele extends OpMode {
    public TeleHandler teleHandler;
    @Override
    public void init() {
        teleHandler = new TeleHandler(gamepad1, gamepad1,hardwareMap, telemetry);
        teleHandler.init();
    }

    @Override
    public void loop() {
        teleHandler.update();
    }
}
