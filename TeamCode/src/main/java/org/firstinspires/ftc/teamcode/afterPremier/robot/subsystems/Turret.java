package org.firstinspires.ftc.teamcode.afterPremier.robot.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.afterPremier.util.Alliance;
import org.firstinspires.ftc.teamcode.afterPremier.util.RobotConstants;

@Configurable
public class Turret {
    private final DcMotorEx t;
    private double encoderPositionOffsetTicks;
    private double manualAngleOffsetRadians;
    public Turret(HardwareMap hardwareMap){
        t = hardwareMap.get(DcMotorEx.class, "turret");
        t.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        t.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //f = follower;
        encoderPositionOffsetTicks = 0;
        manualAngleOffsetRadians = 0;
    }
    private static final double TICKS_PER_REV = 145.1;
    private static final double TURRET_GEAR_RATIO = 5.1;
    private static final double TICKS_PER_RADIAN = (TICKS_PER_REV * TURRET_GEAR_RATIO) / (Math.PI * 2);
    private static final double ENCODER_RESET_THRESHOLD_TICKS = 5;

    public void aim(Pose goal, Pose current){
        double goalAngle = Math.atan2(goal.getY() - current.getY(), goal.getX() - current.getX());
        double robotHeading = current.getHeading();
        // Use the shortest signed angle so mirrored blue headings near the +/-pi wrap
        // don't send the turret to the opposite hard stop.
        double turretAngle = MathFunctions.normalizeAngleSigned(goalAngle - robotHeading) + manualAngleOffsetRadians;
        if(turretAngle >= Math.PI/2){
            turretAngle = Math.PI/2;
        }
        if(turretAngle <= -Math.PI/2){
            turretAngle = -Math.PI/2;
        }
        double targetPos = turretAngle * TICKS_PER_RADIAN;
        turnTurret(targetPos);
    }
    public double getTurretOffset(){
        return encoderPositionOffsetTicks;
    }
    //sets offsets
    public void setOffset(double newOffset){
        encoderPositionOffsetTicks = newOffset;
    }
    //call at beginning of tele
    public void useLastTurretPos(){
        if(Math.abs(t.getCurrentPosition()) < ENCODER_RESET_THRESHOLD_TICKS){
            setOffset(RobotConstants.turretPosTransfer);
        }
        else{
            setOffset(0);
        }
    }
    public int getPosition(){
        return t.getCurrentPosition();
    }
    //adds manual offsets by a little in either direction
    public CommandBuilder incrementLeft(){
        return instant(() -> manualAngleOffsetRadians += 0.1);
    }
    public CommandBuilder incrementRight(){
        return instant(() -> manualAngleOffsetRadians -= 0.1);
    }
    private final ElapsedTime turretTime= new ElapsedTime();
    private double lastTurretError = 0;

    public void turnTurret(double tPosition) {
        double cPosition = t.getCurrentPosition() + getTurretOffset();
        double error = tPosition - cPosition;

        double dt = turretTime.seconds();
        if (dt < 0.0001) dt = 0.0001;
        double derivative = (error - lastTurretError) / dt;

        lastTurretError = error;

        turretTime.reset();

        double output = (error * TurretConstants.Kp);

        t.setPower(output);
    }
    @SuppressWarnings("FieldMayBeFinal")
    @Configurable
    public static class TurretConstants {
        private static double Kp = 0.03;
        private static double Kf = -0.1;
        private static double Ki = 0;
        public static final double overrideSensitivity = 10;
        private static double angleMultiplier = 0.01;
        private static double magnitudeMultiplier = 0.01;
        private static double lowPassAlpha = 0.2;
        private static double maxChange = 0.05;
    }
}
