package org.firstinspires.ftc.teamcode.RIstates.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "quickyshootytesty")
@Configurable
public class QuickShootTest extends OpMode {
    public DcMotorEx flyTop;
    public DcMotorEx flyBottom;
    public ServoImplEx hood;
    public double target = 1650;
    TelemetryManager telemetryManager;
    @Override
    public void init() {
        flyTop = hardwareMap.get(DcMotorEx.class, "ft");
        flyBottom = hardwareMap.get(DcMotorEx.class, "fb");
        hood = hardwareMap.get(ServoImplEx.class, "h");
        flyTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyTop.setDirection(DcMotorEx.Direction.REVERSE);
        flyTop.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flyBottom.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
    }
    @Override
    public void loop() {
        telemetryManager.update();
        if(gamepad2.b){
            updateRight(target);
            updateLeft(target);
        }
        else{
            flyBottom.setPower(0);
            flyTop.setPower(0);
        }
        if(gamepad2.a){
            hood.setPosition(hood.getPosition() + 0.01);
        }
        if(gamepad2.x){
            hood.setPosition(hood.getPosition() - 0.01);
        }
    }
    ElapsedTime pidTime = new ElapsedTime();
    public double integralSum;
    public double lastError;
    public double Kp = 0.004;
    public double Kd = 0;
    public double Ki = 0;
    public double Kf = 0.006;
    public void updateRight(double target){
        double error = target-(flyTop.getVelocity());
        double dt = pidTime.seconds();
        if (dt < 0.0001) dt = 0.0001;
        integralSum += error* dt;
        double derivative = (error- lastError)/ dt;
        lastError = error;

        pidTime.reset();


        double outputRight = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (target * Kf);

        flyTop.setPower(outputRight);
        //flyLeft.setPower(output); //in case we switch back to one pid
    }
    ElapsedTime pidLeft = new ElapsedTime();
    double lastErrorLeft;
    double integralSumLeft;
    public double outputLeft = 0; // basically the same as the normal PIDControl
    public void updateLeft(double target){
        double error = target-(flyBottom.getVelocity());
        double dt = pidLeft.seconds();
        if (dt < 0.0001) dt = 0.0001;
        integralSumLeft += error* dt;
        double derivative = (error- lastErrorLeft)/ dt;
        lastErrorLeft = error;

        pidLeft.reset();


        outputLeft = (error * Kp) + (derivative * Kd) + (integralSumLeft * Ki) + (target * Kf);

        flyBottom.setPower(outputLeft);
    }
}
