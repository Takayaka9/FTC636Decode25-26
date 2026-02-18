package org.firstinspires.ftc.teamcode.zRIstates.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
@Configurable
public class HoodTester extends OpMode {
    Servo h;
    private static double start = 1;
    public static double position = 1;
    @Override
    public void init() {
        h = hardwareMap.get(Servo.class, "hood");
        position = start;
    }

    @Override
    public void loop() {
        h.setPosition(position);
    }
}
