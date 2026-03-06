package org.firstinspires.ftc.teamcode.NewEnglands.opModes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Configurable
@TeleOp
public class KickstandTest extends OpMode {
    Servo liftLeft;
    Servo liftRight;
    public static double liftLeftPos;
    public static double liftRightPos;
    @Override
    public void init() {
        liftLeft = hardwareMap.get(Servo.class, "liftLeft");
        liftRight = hardwareMap.get(Servo.class, "liftRight");
    }

    @Override
    public void loop() {
        liftLeft.setPosition(liftLeftPos);
        liftRight.setPosition(liftRightPos);
    }
}
