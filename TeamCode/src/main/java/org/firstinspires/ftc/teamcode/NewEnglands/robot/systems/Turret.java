package org.firstinspires.ftc.teamcode.NewEnglands.robot.systems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.NewEnglands.utils.alliance.Alliance;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.alliance.GetTargetDistance;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.BaseSubsystem;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.CommandLoop;

public class Turret extends BaseSubsystem {
    DcMotorEx turret;
    private final Follower follower;
    private final Gamepad gamepad2;
    public Turret(CommandLoop maps, HardwareMap hardwareMap, Follower follower, Gamepad gamepad2) {
        super(maps);
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.follower = follower;
        this.gamepad2 = gamepad2;
    }
    public static final double TICKS_PER_REV = 145.1;
    public double goalAngle;
    public double ticksToMove;
    /* Function to move the turret to a certain angle
    Requires color (1 for blue, 2 for red) and follower object
    Calls turnTurret with required inputs to move the turret
     */
    public void trackGoal(Alliance alliance){
        if(alliance == Alliance.UNSELECTED){
            ticksToMove = (gamepad2.left_stick_x * 300);
            turnTurret(ticksToMove);
            return;
        }
        if(alliance == Alliance.BLUE){
            goalAngle = Math.atan2(GetTargetDistance.goalPoses.blueY - follower.getPose().getY(), GetTargetDistance.goalPoses.blueX - follower.getPose().getX());
        }
        if(alliance == Alliance.RED){
            goalAngle = Math.atan2(GetTargetDistance.goalPoses.redY - follower.getPose().getY(), GetTargetDistance.goalPoses.redX - follower.getPose().getX());
        }
        double robotHeading = follower.getHeading();
        double turretAngle = goalAngle - robotHeading;
        if(turretAngle >= Math.PI/2){
            turretAngle = Math.PI/2;
        }
        if(turretAngle <= -Math.PI/2){
            turretAngle = -Math.PI/2;
        }

        ticksToMove = (turretAngle*((TICKS_PER_REV*5.1)/(Math.PI*2)));
        turnTurret(ticksToMove + getOffset(alliance));
    }
    @Configurable
    public static class TurretPID {
        public static double Kp = 0.03;
        public static double Kd = 0;
        public static double Ki = 0;
        public static double I_MAX = 500;
    }
    ElapsedTime turretTime = new ElapsedTime();
    double lastTurretError;
    double turretIntegral;
    double output;
    //PID to turn turret
    public void turnTurret(double tPosition){
        double cPosition = turret.getCurrentPosition(); //TODO: change 0 to getPosition
        double error = tPosition - cPosition;
        double dt = turretTime.seconds();
        if (dt < 0.0001) dt = 0.0001;
        turretIntegral = error * dt;
        //turretIntegral = Math.max(-I_MAX, Math.min(I_MAX, turretIntegral));
        double derivative = (error- lastTurretError)/ dt;
        lastTurretError = error;

        turretTime.reset();


        output = (error * TurretPID.Kp) + (derivative * TurretPID.Kd) + (turretIntegral * TurretPID.Ki) ;

        turret.setPower(output);

        //telemetryM.addData("current position", cPosition);
        //telemetryM.addData("turret desired position", tPosition);
        //telemetryM.addData("turret motor power", Math.max(-1, Math.min(1, output)));
    }
    GetTargetDistance getTargetDistance;
    Pose lastPose;
    double lastDistance;
    public double angleMultiplier = 1;
    public double magnitudeMultiplier = 1;
    public double getOffset(Alliance alliance){
        double angleGoal;

        //inches per second the bot is moving at
        double magnitude = follower.getVelocity().getMagnitude();

        //what direction the bot is moving in
        double velAngle = follower.getVelocity().getTheta();

        //get the angle to the goal from bot pos, same as turret aiming
        if(alliance == Alliance.BLUE){
            angleGoal = Math.atan2(GetTargetDistance.goalPoses.blueY - follower.getPose().getY(), GetTargetDistance.goalPoses.blueX - follower.getPose().getX());
        }
        else if(alliance == Alliance.RED){
            angleGoal = Math.atan2(GetTargetDistance.goalPoses.redY - follower.getPose().getY(), GetTargetDistance.goalPoses.redX - follower.getPose().getX());
        }
        else{
            angleGoal = 0;
        }

        //calculate how much the bot is moving laterally to the goal (further from direct line to goal = more lateral movement)
        double angle = Math.abs(angleGoal - velAngle);

        //total velocity relative to the goal (sorta, the values are messed up but it's chill)
        double magGoal = (angleMultiplier*angle)*(magnitude*magnitudeMultiplier);

        //return pos/neg values depending on moving left or right
        if(velAngle > angleGoal){
            return magGoal;
        }
        return -magGoal;
    }

    public double turretPosition(){
        return turret.getCurrentPosition();
    }
}
