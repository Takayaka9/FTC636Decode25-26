package org.firstinspires.ftc.teamcode.zRIstates.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp
@Configurable
public class StopperTester extends OpMode {
    Servo h;
    public static double position = 0;
    @Override
    public void init() {
        h = hardwareMap.get(Servo.class, "gate");
        //position = 0;
    }

    @Override
    public void loop() {
        h.setPosition(position);
    }
}
