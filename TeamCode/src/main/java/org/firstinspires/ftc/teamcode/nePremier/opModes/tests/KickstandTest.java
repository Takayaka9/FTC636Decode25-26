package org.firstinspires.ftc.teamcode.nePremier.opModes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Configurable
@TeleOp
public class KickstandTest extends OpMode {
    Servo liftLeft = null;
    Servo liftRight = null;
    public static double leftPos = 1;
    //public static double rightPos = 0;

    @Override
    public void init() {
        liftLeft = hardwareMap.get(Servo.class, "liftLeft");
        liftRight = hardwareMap.get(Servo.class, "liftRight");
    }

    @Override
    public void loop() {
        liftLeft.setPosition(leftPos);
        liftRight.setPosition(1-leftPos);
    }
}
