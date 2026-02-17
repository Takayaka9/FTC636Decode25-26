package org.firstinspires.ftc.teamcode.robot.systems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.systems.servos.HoodServo;
import org.firstinspires.ftc.teamcode.utils.alliance.Alliance;
import org.firstinspires.ftc.teamcode.utils.alliance.GetTargetDistance;
import org.firstinspires.ftc.teamcode.utils.commandUtils.BaseCommand;
import org.firstinspires.ftc.teamcode.utils.commandUtils.BaseSubsystem;
import org.firstinspires.ftc.teamcode.utils.commandUtils.CommandLoop;

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
        turnTurret(ticksToMove);
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

    public double turretPosition(){
        return turret.getCurrentPosition();
    }

}
