package org.firstinspires.ftc.teamcode.states.subsystems;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
public class Turret {
    //TelemetryManager telemetryM;
    DcMotorEx turret;
    public Turret(HardwareMap hardwareMap, String name){
        turret = hardwareMap.get(DcMotorEx.class, name);
    }
    //turret code!
    //TODO: Ticks per Rev incorrect, thats for a 6000
    public static final double TICKS_PER_REV = 28;
    public static final double BLUE_GOAL_Y = 138;
    public static final double BLUE_GOAL_X = 0;
    public static final double RED_GOAL_Y = 138;
    public static final double RED_GOAL_X = 138;
    double goalAngle;

    /* Function to move the turret to a certain angle
    Requires color (1 for blue, 2 for red) and follower object
    Calls turnTurret with required inputs to move the turret
     */
    public void trackGoal(int color, Follower follower){
        //follower = Constants.createFollower(hardwareMap);
        //follower.update();
        if(color == 0){
            return;
        }
        if(color == 1){
            goalAngle = Math.atan2(BLUE_GOAL_Y - follower.getPose().getY(), BLUE_GOAL_X - follower.getPose().getX());
        }
        if(color == 2){
            goalAngle = Math.atan2(RED_GOAL_Y - follower.getPose().getY(), RED_GOAL_X - follower.getPose().getX());
        }
        double robotHeading = follower.getHeading();
        double turretAngle = goalAngle - robotHeading;

        double ticksToMove = turretAngle*(TICKS_PER_REV/6.2832);
        turnTurret(ticksToMove);
    }

    ElapsedTime turretTime = new ElapsedTime();
    double lastTurretError;
    double turretIntegral;
    public static double turretKp = 0.01;
    public static double turretKd = 0;
    public static double turretKi = 0;
    public static double I_MAX = 500;

    /*
   turnTurret is a method to move the turret using PID + FF(?)
   inputs: tPosition (desired turret position in encoder ticks)
   outputs: targetDistance (also printed to panels)
   !! It is never needed to call this method - it is called in trackGoal !!
    */
    public void turnTurret(double tPosition){
        double cPosition = turret.getCurrentPosition(); //TODO: change 0 to getPosition
        double error = tPosition - cPosition;
        double dt = turretTime.seconds();
        if (dt < 0.0001) dt = 0.0001;
        turretIntegral += error* dt;
        //turretIntegral = Math.max(-I_MAX, Math.min(I_MAX, turretIntegral));
        double derivative = (error- lastTurretError)/ dt;
        lastTurretError = error;

        turretTime.reset();

        double output;
        output = (error * turretKp) + (derivative * turretKd) + (turretIntegral * turretKi) ;

        //TODO: need to change this to whatever we name the motor
        //turret.setPower(output);

        //telemetryM.addData("current position", cPosition);
        //telemetryM.addData("turret desired position", tPosition);
        //telemetryM.addData("turret motor power", Math.max(-1, Math.min(1, output)));
    }

    public double turretPosition(){
        return turret.getCurrentPosition();
    }

}
