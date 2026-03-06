package org.firstinspires.ftc.teamcode.NewEnglands.opModes.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class KickstandTest extends OpMode {
    Servo lift1;
    Servo lift2;
    public static double lift1Pos;
    public static double lift2Pos;
    @Override
    public void init() {
        lift1 = hardwareMap.get(Servo.class, "lift1");
        lift2 = hardwareMap.get(Servo.class, "lift2");
    }

    @Override
    public void loop() {
        lift1.setPosition(lift1Pos);
        lift2.setPosition(lift2Pos);
    }
}
