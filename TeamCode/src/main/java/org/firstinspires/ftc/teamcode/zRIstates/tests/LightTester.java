package org.firstinspires.ftc.teamcode.zRIstates.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@Configurable
@TeleOp
public class LightTester extends OpMode {
    Servo light;
    TelemetryManager telemetryManager;
    public static double color = 0;
    public static double color2 = 0;
    boolean y = false;
    boolean pressY = false;
    boolean b = false;
    boolean pressB = false;
    @Override
    public void init() {
        light = hardwareMap.get(Servo.class, "l");
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void loop() {
        telemetryManager.update();
        if(gamepad2.y){
            light.setPosition(color);
        }
        if(gamepad2.b){
            light.setPosition(color2);
        }
    }
}
