package org.firstinspires.ftc.teamcode.nePremier.robot.systems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.nePremier.utils.commandUtils.BaseSubsystem;

public class TakaShooter extends BaseSubsystem {
    @Configurable
    private static class shooterTune {
        public static double Kp = 0.01;
        public static double Ki = 0.0005;
        public static double Kd = 0;
        public static double Kv = 0.00069;
        public static double Ks = 0.155;
        public static double minActiveTps = 900;
        public static double integralLimit = 2000;
        //public static double Kf = 0.00036;
        static double d1 = 36; static double r1 = 900;
        static double d2 = 50; static double r2 = 900;
        static double d3 = 75; static double r3 = 1000;
        static double d4 = 96; static double r4 = 1150;
        static double d5 = 108; static double r5 = 1600;
        static double d6 = 150; static double r6 = 1600;
        public static double brake = -0.3;
    }
    private final InterpLUT lut = new InterpLUT();
    private final DcMotorEx shooter1;
    private final DcMotorEx shooter2;

    public TakaShooter(HardwareMap hardwareMap) {
        super();
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setDirection(DcMotorEx.Direction.REVERSE);
        shooter1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lut.add(0, shooterTune.r1);
        lut.add(shooterTune.d1, shooterTune.r1);
        lut.add(shooterTune.d2, shooterTune.r2);
        lut.add(shooterTune.d3, shooterTune.r3);
        lut.add(shooterTune.d4, shooterTune.r4);
        lut.add(shooterTune.d5, shooterTune.r5);
        lut.add(shooterTune.d6, shooterTune.r6);
        lut.add(1000, shooterTune.r6);
        lut.createLUT();
    }



    //PID 1
    private final ElapsedTime pidTime1 = new ElapsedTime();
    private double integralSum1 = 0;
    private double lastError1 = 0;
    private double output1 = 0;
    private void update1(double target) {
        output1 = updateShooterMotor(shooter1, target, pidTime1, true);
    }

    //PID 2
    private final ElapsedTime pidTime2 = new ElapsedTime();
    private double lastError2 = 0;
    private double integralSum2 = 0;
    private double output2 = 0;
    private void update2(double target) {
        output2 = updateShooterMotor(shooter2, target, pidTime2, false);
    }

    private double updateShooterMotor(DcMotorEx shooter, double target, ElapsedTime pidTime, boolean isShooterOne) {
        if (target < shooterTune.minActiveTps) {
            resetControllerState(isShooterOne, pidTime);
            shooter.setPower(0);
            return 0;
        }

        double error = target - shooter.getVelocity();
        double dt = Math.max(pidTime.seconds(), 0.0001);

        double integralSum = (isShooterOne ? integralSum1 : integralSum2) + (error * dt);
        integralSum = clamp(integralSum, -shooterTune.integralLimit, shooterTune.integralLimit);

        double lastError = isShooterOne ? lastError1 : lastError2;
        double derivative = (error - lastError) / dt;
        double feedForward = shooterTune.Ks + (target * shooterTune.Kv);
        double output = feedForward
                + (error * shooterTune.Kp)
                + (integralSum * shooterTune.Ki)
                + (derivative * shooterTune.Kd);
        output = clamp(output, 0, 1);

        if (isShooterOne) {
            integralSum1 = integralSum;
            lastError1 = error;
        } else {
            integralSum2 = integralSum;
            lastError2 = error;
        }

        pidTime.reset();
        shooter.setPower(output);
        return output;
    }

    private void resetControllerState(boolean isShooterOne, ElapsedTime pidTime) {
        if (isShooterOne) {
            integralSum1 = 0;
            lastError1 = 0;
            output1 = 0;
        } else {
            integralSum2 = 0;
            lastError2 = 0;
            output2 = 0;
        }
        pidTime.reset();
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }



    /// MAIN RUN METHOD
    public void runForDistance(double distance){
        update1(getShooterTPS(distance));
        update2(getShooterTPS(distance));
    }
    /// SET POWER ZERO (DO NOT CALL RUN FOR DISTANCE WHEN USING THIS)
    public void stop(){
        resetControllerState(true, pidTime1);
        resetControllerState(false, pidTime2);
        shooter1.setPower(0);
        shooter2.setPower(0);
    }
    /// REVERSES THE SHOOTER
    public void reverse() {
        resetControllerState(true, pidTime1);
        resetControllerState(false, pidTime2);
        shooter1.setPower(-1);
        shooter2.setPower(-1);
    }
    /// SLIGHT REVERSE FOR BRAKING
    /// SETS POWER TO BRAKE POWER
    public void brake(){
        resetControllerState(true, pidTime1);
        resetControllerState(false, pidTime2);
        shooter1.setPower(shooterTune.brake);
        shooter2.setPower(shooterTune.brake);
    }
    /// TEST METHOD FOR PID
    public void test(double tps) {
        update2(tps);
        update1(tps);
    }
    /// CALC NEEDED TPS WITH LUT
    public int getShooterTPS(double targetDistance){
        return (int) Math.round(lut.get(targetDistance));
    }
    /// RETRIEVES AVERAGE VELOCITY OF BOTH SHOOTERS
    public int getAverageVelocity() {
        double averageVelocity = ((shooter1.getVelocity() + shooter2.getVelocity()) / 2);
        return (int) (Math.round(averageVelocity));
    }
    /// SHOOTER 1 TPS
    public int getShooter1tps() {
        return (int) Math.round(shooter1.getVelocity());
    }
    /// SHOOTER 2 TPS
    public int getShooter2tps() {
        return (int) Math.round(shooter2.getVelocity());
    }
    /// SET POWER FOR MOTOR ONE
    public double get1Power() {
        return output1;
    }
    /// SET POWER FOR MOTOR TWO
    public double get2Power() {
        return output2;
    }
    /// LAST ERROR
    public double getError(){
        //return getShooterTPS(distance) - shooter1.getVelocity();
        return lastError1;
    }

}
