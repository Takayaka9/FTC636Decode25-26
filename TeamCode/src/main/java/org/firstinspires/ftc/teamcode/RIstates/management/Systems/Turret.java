package org.firstinspires.ftc.teamcode.RIstates.management.Systems;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class Turret {
    //TelemetryManager telemetryM;
    public DcMotorEx turret;
    Follower follower;
    public int offset = 0;
    public Turret(HardwareMap hardwareMap, Follower follower, String name){
        turret = hardwareMap.get(DcMotorEx.class, name);
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.follower = follower;
    }
    //turret code!
    public static final double TICKS_PER_REV = 145.1;
    //20to102
    public static double BLUE_GOAL_Y = 144;
    public static double BLUE_GOAL_X = 4;
    public static double RED_GOAL_Y = 140;
    public static double RED_GOAL_X = 144;
    public double goalAngle;
    public double ticksToMove;


    /* Function to move the turret to a certain angle
    Requires color (1 for blue, 2 for red) and follower object
    Calls turnTurret with required inputs to move the turret
     */
    public void trackGoal(int color){
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
        if(turretAngle >= Math.PI/2){
            turretAngle = Math.PI/2;
        }
        if(turretAngle <= -Math.PI/2){
            turretAngle = -Math.PI/2;
        }

        ticksToMove = (turretAngle*((TICKS_PER_REV*5.1)/(Math.PI*2)));// + offset;
        turnTurret(ticksToMove);
    }




    /*
   turnTurret is a method to move the turret using PID + FF(?)
   inputs: tPosition (desired turret position in encoder ticks)
   outputs: targetDistance (also printed to panels)
   !! It is never needed to call this method - it is called in trackGoal !!
    */
    double output;
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


        output = (error * turretKp) + (derivative * turretKd) + (turretIntegral * turretKi) ;

        turret.setPower(output);

        //telemetryM.addData("current position", cPosition);
        //telemetryM.addData("turret desired position", tPosition);
        //telemetryM.addData("turret motor power", Math.max(-1, Math.min(1, output)));
    }

    public double turretPosition(){
        return turret.getCurrentPosition();
    }

}
