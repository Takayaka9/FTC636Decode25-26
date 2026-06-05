package org.firstinspires.ftc.teamcode.nePremier.opModes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.nePremier.tele.TeleControls;

import java.util.List;

@TeleOp(name = "ForDemo")
public class PatrickInLondonTele extends OpMode {
    public TeleControls teleControls = null;
    List <LynxModule> allHubs;
    @Override
    public void init() {
        teleControls = new TeleControls(gamepad1, gamepad2, hardwareMap, telemetry);
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    @Override
    public void start() {
        teleControls.start();
    }

    @Override
    public void loop() {
        teleControls.update();
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    @Override
    public void stop() {
        teleControls.stop();
    }
}
