package org.firstinspires.ftc.teamcode.RIstates.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@Configurable
@TeleOp(name = "New Shooter Test", group = "TeleOp")
public class NewShooterTester extends OpMode {
    public DcMotorEx flyTop;
    public DcMotorEx flyBottom;
    public ServoImplEx hood;
    public static double target = 950;
    Follower follower;
    TelemetryManager telemetryManager;
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        flyTop = hardwareMap.get(DcMotorEx.class, "sr");
        flyBottom = hardwareMap.get(DcMotorEx.class, "sl");
        hood = hardwareMap.get(ServoImplEx.class, "hood");
        flyTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyTop.setDirection(DcMotorEx.Direction.REVERSE);
        flyTop.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flyBottom.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
        //hood.setPosition(0);
    }
    Pose start = new Pose(144, 0);
    @Override
    public void start() {
        follower.setStartingPose(start);
    }

    @Override
    public void loop() {
        follower.update();
        telemetryManager.update();
        getTargetDistance(follower, 2);
        if(gamepad2.b) {
            updateRight(target);
            updateLeft(target);
        }
        else{
            flyBottom.setPower(0);
            flyTop.setPower(0);
        }


        /*
        if(gamepad2.a){
            hood.setPosition(hood.getPosition() + 0.0001);
        }
        if(gamepad2.x){
            hood.setPosition(hood.getPosition() - 0.0001);
        }

         */
        telemetryManager.addData("top power", flyTop.getPower());
        telemetryManager.addData("bottom power", flyBottom.getPower());
        telemetryManager.addData("top velocity", flyTop.getVelocity());
        telemetryManager.addData("bottom velocity", flyBottom.getVelocity());
        telemetryManager.addData("servo pos", hood.getPosition());
        telemetryManager.addData("distance", targetDistance);
        telemetryManager.debug("target", target);
    }
    ElapsedTime pidTime = new ElapsedTime();
    public double integralSum;
    public double derivative;
    public double lastError;
    public static double Kp = 0.004;
    public static double Kd = 0;
    public static double Ki = 0;
    public static double Kf = 0.003;
    public void updateRight(double target){
        double currentVelocity = flyTop.getVelocity();
        double error = target - currentVelocity;
        double output = (Kp * error)
                + (Kf * target);
        flyTop.setPower(output);
        /*double error = target-(flyTop.getVelocity());
        double dt = pidTime.seconds();
        if (dt < 0.0001) dt = 0.0001;
        integralSum += error* dt;
        double derivative = (error- lastError)/ dt;
        lastError = error;

        pidTime.reset();


        double outputRight = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (target * Kf);

        flyTop.setPower(outputRight);
        //flyLeft.setPower(output); //in case we switch back to one pid

         */
    }
    ElapsedTime pidLeft = new ElapsedTime();
    double lastErrorLeft;
    double integralSumLeft;
    public double outputLeft = 0; // basically the same as the normal PIDControl
    public void updateLeft(double target){
        double currentVelocity = flyBottom.getVelocity();
        double error = target - currentVelocity;
        double output = (Kp * error)
                + (Kf * target);
        flyBottom.setPower(output);
    }

    private final Pose blueGoal = new Pose(0, 138);
    private final Pose redGoal = new Pose(138, 138);
    public static double targetDistance = 0;
    public double getTargetDistance(Follower follower, int alliance){
        if (alliance == 1){
            Pose currentPose = follower.getPose();
            targetDistance = currentPose.distanceFrom(blueGoal);
        }
        else if (alliance == 2){
            Pose currentPose = follower.getPose();
            targetDistance = currentPose.distanceFrom(redGoal);
        }
        if (targetDistance > 20) {
            return targetDistance;
        } else {
            return 1;
        }
    }
}
