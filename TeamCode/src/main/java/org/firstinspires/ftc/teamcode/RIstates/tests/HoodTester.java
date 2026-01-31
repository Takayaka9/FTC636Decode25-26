package org.firstinspires.ftc.teamcode.RIstates.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class HoodTester extends OpMode {
    Servo h;
    public static double position = 0;
    @Override
    public void init() {
        h = hardwareMap.get(Servo.class, "hood");
        position = 0;
    }

    @Override
    public void loop() {
        if(gamepad2.a){

        }
    }
}
