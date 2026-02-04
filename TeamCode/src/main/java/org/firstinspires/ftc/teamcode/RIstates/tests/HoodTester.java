package org.firstinspires.ftc.teamcode.RIstates.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
@Configurable
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
            h.setPosition(position);
        }
    }
}
