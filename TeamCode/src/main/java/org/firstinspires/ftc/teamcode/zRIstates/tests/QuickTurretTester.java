package org.firstinspires.ftc.teamcode.zRIstates.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Configurable
@TeleOp
public class QuickTurretTester extends OpMode {
    public DcMotorEx turret;
    TelemetryManager telemetryManager;
    public static double targetPosition;
    @Override
    public void init() {
        turret = hardwareMap.get(DcMotorEx.class, "t");
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void loop() {
        telemetryManager.update();
        update();
    }
    public static double kP = 0.1;
    public static double kI = 0;
    public static double kD = 0;
    ElapsedTime timer = new ElapsedTime();
    public static double integralSum = 0;
    double lastError;
    public void update(){
        int currentPosition = turret.getCurrentPosition();
        double error = targetPosition - currentPosition;

        double dt = timer.seconds();
        timer.reset();

        // Integral (anti-windup)
        integralSum += error * dt;

        // Derivative
        double derivative = (error - lastError) / dt;
        lastError = error;

        double output =
                (kP * error) +
                        (kI * integralSum) +
                        (kD * derivative);

        turret.setPower(output);
    }
}
