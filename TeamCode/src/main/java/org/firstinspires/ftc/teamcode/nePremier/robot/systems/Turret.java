package org.firstinspires.ftc.teamcode.nePremier.robot.systems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.Alliance;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.CurrentAlliance;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.TDistHelper;
import org.firstinspires.ftc.teamcode.nePremier.utils.commandUtils.BaseSubsystem;

import java.util.ArrayList;

public class Turret extends BaseSubsystem {
    private final DcMotorEx turret;
    private final Follower follower;
    public Turret(HardwareMap hardwareMap, Follower follower) {
        super();
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.follower = follower;
    }
    private static final double TICKS_PER_REV = 145.1;
    private double goalAngle = 0;

    /**
     *  Function to move the turret to a certain angle
     *  Requires color (1 for blue, 2 for red) and follower object
     *  Calls turnTurret with required inputs to move the turret
     */
    public void trackGoal(){
        double targetPos;
        Alliance alliance = CurrentAlliance.alliance;
        if(alliance == Alliance.BLUE){
            goalAngle = Math.atan2(TDistHelper.goalPoses.blueY - follower.getPose().getY(), TDistHelper.goalPoses.blueX - follower.getPose().getX());
        }
        if(alliance == Alliance.RED){
            goalAngle = Math.atan2(TDistHelper.goalPoses.redY - follower.getPose().getY(), TDistHelper.goalPoses.redX - follower.getPose().getX());
        }
        double robotHeading = follower.getHeading();
        double turretAngle = (goalAngle - robotHeading);// + getOffset();
        if(turretAngle >= Math.PI/2){
            turretAngle = Math.PI/2;
        }
        if(turretAngle <= -Math.PI/2){
            turretAngle = -Math.PI/2;
        }

        targetPos = (turretAngle*((TICKS_PER_REV*5.1)/(Math.PI*2)));
        turnTurret(targetPos);
    }

    @SuppressWarnings("FieldMayBeFinal")
    @Configurable
    public static class TurretConstants {
        private static double Kp = 0.03;
        private static double Kd = 0;
        private static double Ki = 0;
        public static final double overrideSensitivity = 10;
        private static double angleMultiplier = 0.01;
        private static double magnitudeMultiplier = 0.01;
        private static double lowPassAlpha = 0.2;
        private static double maxChange = 0.05;
    }

    //PID to turn turret
    private final ElapsedTime turretTime = new ElapsedTime();
    private double lastTurretError = 0;
    public void turnTurret(double tPosition){
        double cPosition = turret.getCurrentPosition(); //TODO: change 0 to getPosition
        double error = tPosition - cPosition;
        double dt = turretTime.seconds();
        if (dt < 0.0001) dt = 0.0001;
        double turretIntegral = error * dt;
        //turretIntegral = Math.max(-I_MAX, Math.min(I_MAX, turretIntegral));
        double derivative = (error- lastTurretError)/ dt;
        lastTurretError = error;

        turretTime.reset();


        double output = (error * TurretConstants.Kp) + (derivative * TurretConstants.Kd) + (turretIntegral * TurretConstants.Ki) ;

        turret.setPower(output);
    }
    // taka moving average filter
    // private final ArrayList<Double> lowPassFilter = new ArrayList<>(5);
    private double filteredMagGoal = 0.0;
    private boolean filteredMagGoalInit = false;
    private double getOffset(){
        double angleGoal;

        //inches per second the bot is moving at
        double magnitude = follower.getVelocity().getMagnitude();

        //what direction the bot is moving in
        double velAngle = follower.getVelocity().getTheta();

        //get the angle to the goal from bot pos, same as turret aiming
        if(CurrentAlliance.alliance == Alliance.BLUE){
            angleGoal = Math.atan2(TDistHelper.goalPoses.blueY - follower.getPose().getY(), TDistHelper.goalPoses.blueX - follower.getPose().getX());
        }
        else if(CurrentAlliance.alliance == Alliance.RED){
            angleGoal = Math.atan2(TDistHelper.goalPoses.redY - follower.getPose().getY(), TDistHelper.goalPoses.redX - follower.getPose().getX());
        }
        else{
            angleGoal = 0;
        }

        //calculate how much the bot is moving laterally to the goal (further from direct line to goal = more lateral movement)
        double angle = angleGoal - velAngle;

        //total velocity relative to the goal (sorta, the values are messed up but it's chill)
        double magGoal = (TurretConstants.angleMultiplier*angle)*(magnitude*TurretConstants.magnitudeMultiplier);

        //low pass filter
        if (!filteredMagGoalInit) {
            filteredMagGoal = magGoal;
            filteredMagGoalInit = true;
        } else {
            double alpha = TurretConstants.lowPassAlpha;
            filteredMagGoal = (alpha * magGoal) + ((1.0 - alpha) * filteredMagGoal);
        }

        //emad simple filter for max change
//        double delta = magGoal - filteredMagGoal;
//        delta = Math.max(-TurretConstants.maxChange, Math.min(TurretConstants.maxChange, delta));
//        filteredMagGoal += delta;

        //emad other simpler filter
//        if (Math.abs(magGoal) < TurretConstants.maxChange) {
//            magGoal = 0;
//        }


        // taka moving  average filter
        // if (lowPassFilter.size() > 4) {
        //     lowPassFilter.remove(0);
        // }
        // lowPassFilter.add(magGoal);
        // magGoal = averageList(lowPassFilter);

        //return pos/neg values depending on moving left or right
//        if(velAngle > angleGoal){
//            return -magGoal;
//        }
        return magGoal;
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
}
