package org.firstinspires.ftc.teamcode.nePremier.robot.systems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.nePremier.robot.systems.turret.TurretI;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.Alliance;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.CurrentAlliance;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.LocalizationHelper;
import org.firstinspires.ftc.teamcode.nePremier.utils.commandUtils.BaseSubsystem;
import org.firstinspires.ftc.teamcode.nePremier.utils.filters.LowPassFilter;
import com.pedropathing.math.MathFunctions;

import java.util.ArrayList;

public class Turret extends BaseSubsystem implements TurretI {
    private final DcMotorEx turret;
    private final Follower follower;
    private final BotPose botPose;
    public Turret(HardwareMap hardwareMap, Follower follower, BotPose botPose) {
        super();
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.botPose = botPose;
        this.follower = follower;
    }
    private static final double TICKS_PER_REV = 145.1;
    private double goalAngle = 0;
    private double turretAngle = 0;
    private double targetPos;
    /**
     *  Function to move the turret to a certain angle
     *  Requires color (1 for blue, 2 for red) and follower object
     *  Calls turnTurret with required inputs to move the turret
     */
    public void trackGoal(){

        Alliance alliance = CurrentAlliance.alliance;
        Pose predicted = botPose.getBotPose();

        if(alliance == Alliance.BLUE){
            goalAngle = Math.atan2(LocalizationHelper.goalPoses.blueY - predicted.getY(), LocalizationHelper.goalPoses.blueX - predicted.getX());
        }
        if(alliance == Alliance.RED){
            goalAngle = Math.atan2(LocalizationHelper.goalPoses.redY - predicted.getY(), LocalizationHelper.goalPoses.redX - predicted.getX());
        }
        double robotHeading = follower.getHeading();
        // Use the shortest signed angle so mirrored blue headings near the +/-pi wrap
        // don't send the turret to the opposite hard stop.
        turretAngle = MathFunctions.normalizeAngleSigned(goalAngle - robotHeading)
                + follower.getAngularVelocity()*TurretConstants.Kf;
        if(turretAngle >= Math.toRadians(80)){
            turretAngle = Math.toRadians(80);
        }
        if(turretAngle <= -Math.toRadians(80)) {
            turretAngle = -Math.toRadians(80);
        }

        targetPos = (turretAngle*((TICKS_PER_REV*5.1)/(Math.PI*2)));
        turnTurret(targetPos);
    }

    @SuppressWarnings("FieldMayBeFinal")
    @Configurable
    public static class TurretConstants {
        private static double Kp = 0.03;
        private static double Kf = -0.1;
        public static final double overrideSensitivity = 10;
    }

    //PID to turn turret
    private final ElapsedTime turretTime = new ElapsedTime();
    private double lastTurretError = 0;
    public void turnTurret(double tPosition){
        double cPosition = turret.getCurrentPosition(); //TODO: change 0 to getPosition
        double error = tPosition - cPosition;

        double dt = turretTime.seconds();
        if (dt < 0.0001) dt = 0.0001;
        double derivative = (error-lastTurretError)/dt;

        lastTurretError = error;

        turretTime.reset();

        double output = (error * TurretConstants.Kp);


        turret.setPower(output);
    }
    public void turnTurretRad(double radians){
        double tPosition = targetPos = (turretAngle*((TICKS_PER_REV*5.1)/(Math.PI*2)));
        double cPosition = turret.getCurrentPosition(); //TODO: change 0 to getPosition
        double error = tPosition - cPosition;

        double dt = turretTime.seconds();
        if (dt < 0.0001) dt = 0.0001;
        double derivative = (error-lastTurretError)/dt;

        lastTurretError = error;

        turretTime.reset();

        double output = (error * TurretConstants.Kp);


        turret.setPower(output);
    }

    public void resetEncoder(){
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public double averageList(ArrayList<Double> list){
        double sum = 0;
        for(int i = 0; i < list.size(); i++){
            sum += list.get(i);
        }
        return sum / list.size();
    }
    public double turretPosition(){
        return turret.getCurrentPosition();
    }
    double lastTargetPos = 0;
    ElapsedTime angleTime = new ElapsedTime();
    public double getGoalAngleDerivative(){
        double angleChange = targetPos - lastTargetPos;
        double dt = angleTime.seconds();
        if(dt < 0.0001){dt = 0.0001;}
        double derivative = angleChange / dt;


        lastTargetPos = targetPos;
        angleTime.reset();

        return derivative;
    }
}
