package org.firstinspires.ftc.teamcode.nePremier.robot.systems.turret;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.Alliance;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.CurrentAlliance;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.TDistHelper;
import org.firstinspires.ftc.teamcode.nePremier.utils.commandUtils.BaseSubsystem;

public class EmadTurret extends BaseSubsystem  implements TurretI {


    @SuppressWarnings("FieldMayBeFinal")
    @Configurable
    public static class EmadTurretConstants {
//        private static double oKd1 = 0.001;
//        private static double oKd2 = 0.0001;
        private static double SOTM = 1;
        private static double tKp = 0.03;
        private static double tKd = 0.0001;
        public static final double ticksPerRev = 145.1;
    }


    private final DcMotorEx turret;
    private final Follower follower;
    public EmadTurret(HardwareMap hardwareMap, Follower follower) {
        super();
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.follower = follower;
    }

    private double goalAngle = 0;

    /**
     *  Function to move the turret to a certain angle
     *  Requires color (1 for blue, 2 for red) and follower object
     *  Calls turnTurret with required inputs to move the turret
     */
    public void trackGoal() {
        turnTurret(TurretHelper.getFracRads(TurretHelper.clamp(getGoalAngle(getSOTM()))));
    }

    private Pose getSOTM() {
        //get the real current pose from follower
        Pose realPose = follower.getPose();
        double x = realPose.getX();
        double y = realPose.getY();
        double heading = realPose.getHeading();

        //get the velocity data from follower
        double vx = follower.getVelocity().getXComponent();
        double vy = follower.getVelocity().getYComponent();

        //multiply the velocity data by the tuning constant
        vx *= EmadTurretConstants.SOTM;
        vy *= EmadTurretConstants.SOTM;

        //add the multiplied velocity data to the real pose
        x += vx;
        y += vy;

        //create the new pose
        return new Pose(x, y, heading);
    }


    //for SOTM adjusted pose input
    private double getGoalAngle(Pose pose){
        double targetPos;
        Alliance alliance = CurrentAlliance.alliance;
        if(alliance == Alliance.BLUE){
            goalAngle = Math.atan2(TDistHelper.goalPoses.blueY - pose.getY(), TDistHelper.goalPoses.blueX - pose.getX());
        }
        if(alliance == Alliance.RED){
            goalAngle = Math.atan2(TDistHelper.goalPoses.redY - pose.getY(), TDistHelper.goalPoses.redX - pose.getX());
        }
        double turretAngle = (goalAngle - follower.getHeading());// + getOffset();

        return turretAngle;
    }

    //simply returns the normal goal angle from follower.getPose()
    private double getGoalAngle(){
        double targetPos;
        Alliance alliance = CurrentAlliance.alliance;
        if(alliance == Alliance.BLUE){
            goalAngle = Math.atan2(TDistHelper.goalPoses.blueY - follower.getPose().getY(), TDistHelper.goalPoses.blueX - follower.getPose().getX());
        }
        if(alliance == Alliance.RED){
            goalAngle = Math.atan2(TDistHelper.goalPoses.redY - follower.getPose().getY(), TDistHelper.goalPoses.redX - follower.getPose().getX());
        }
        double turretAngle = (goalAngle - follower.getHeading());// + getOffset();

        return turretAngle;
    }

//    //Derivative controller for offset
//    private final ElapsedTime offsetTime = new ElapsedTime();
//    private double lastGoalAngle = 0;
//    private double lastOffset = 0;
//    private double offsetDerivative(double turretAngle) {
//        double derivative1 = (lastGoalAngle - turretAngle) / offsetTime.seconds();
//        double derivative2 = (lastOffset - turretAngle) / offsetTime.seconds();
//        double output = (derivative1 * EmadTurretConstants.oKd1) + (derivative2 * EmadTurretConstants.oKd2);
//
//        lastOffset = output;
//        lastGoalAngle = turretAngle;
//        turretTime.reset();
//        return output;
//    }


    //PID to turn turret to desired position
//    private final ElapsedTime turretTime = new ElapsedTime();
//    private double lastTurretError = 0;
    public void turnTurret(double tPosition){
        //get error
        double error = tPosition - turret.getCurrentPosition();
//        //get time
//        double dt = turretTime.seconds();
//        if (dt < 0.0001) dt = 0.0001;
//
//        //calculate derivative
//        double derivative = (error- lastTurretError)/ dt;
//        lastTurretError = error;

        //calculate output
        double output = (error * EmadTurretConstants.tKp);// + (derivative * EmadTurretConstants.tKd);

        //reset timer and set output
//        turretTime.reset();
        turret.setPower(output);
    }


    //utils:
    /// RESET THE ENCODER
    public void resetEncoder(){
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    /// RETURNS CURRENT POSITION
    public double turretPosition(){
        return turret.getCurrentPosition();
    }
}
