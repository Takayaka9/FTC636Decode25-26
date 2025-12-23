package org.firstinspires.ftc.teamcode.states.subsystems;

import static org.firstinspires.ftc.teamcode.states.StatesTeleOp.TurretModes.BLUE;
import static org.firstinspires.ftc.teamcode.states.StatesTeleOp.TurretModes.RED;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.seattlesolvers.solverslib.util.InterpLUT;
import com.seattlesolvers.solverslib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.states.RobotStates;
import org.firstinspires.ftc.teamcode.states.StatesTeleOp;


@Configurable
public class Hood {
    //TelemetryManager telemetryM;
    private final InterpLUT lut = new InterpLUT();
    private ServoImplEx servo;
    public Hood(HardwareMap hardwareMap, String name){
        servo = hardwareMap.get(ServoImplEx.class, name);
        lut.add(d1, p1);
        lut.add(d2, p2);
        lut.add(d3, p3);
        lut.add(d4, p4);
        lut.add(d5, p5);
        lut.add(d6, p6);
        lut.createLUT();
    }

    /*
    angleHood uses InterpLut to calculate angle of hood based on target distance
    inputs: targetDistance, hardwareMap
    output: panels telemetry and servo position
     */
    double d1 = 0; double p1 = 0;
    double d2 = 0; double p2 = 0;
    double d3 = 0; double p3 = 0;
    double d4 = 0; double p4 = 0;
    double d5 = 0; double p5 = 0;
    double d6 = 0; double p6 = 0;

    public void angleHood(double targetDistance) {
        double angle = lut.get(targetDistance);
        //TODO: figure out what we call this servo
        //servo.setPosition(angle);
        //telemetryM.addData("angle", angle);
    }

    public void passive(){
        //servo.setPosition(passive);
    }

    public void increment(boolean positive){
        if(positive){
            servo.setPosition(servo.getPosition() + 0.03);
        }
        else{
            servo.setPosition(servo.getPosition() - 0.03);
        }
    }


    /*
    shooterRPM uses InterpLut to calculate shooterRPM based on target distance
    inputs: targetDistance, hardwareMap
    output: panels telemetry
    return: desiredRPM

    public int shooterRPM(double targetDistance, HardwareMap hardwareMap){

        robot = new RobotStates(hardwareMap);
        lut.add(d1, r1);
        lut.add(d2, r2);
        lut.add(d3, r3);
        lut.add(d4, r4);
        lut.add(d5, r5);
        lut.add(d6, r6);
        lut.createLUT();

        int calcRPM = (int) Math.round(lut.get(targetDistance));
        telemetryM.addData("Calculated RPM", calcRPM);

        return calcRPM;
    }

     */

    /*
    Get Target Distance is a method to retrieve target distance
    inputs: robotPose, (turret) mode
    outputs: targetDistance (also printed to panels)
    call once in opmode before using methods requiring 'targetDistance' then pass into method

    private final Pose blueGoal = new Pose(0, 138);
    private final Pose redGoal = new Pose(138, 138);

    public double getTargetDistance(Follower follower, StatesTeleOp.TurretModes mode){
        double targetDistance = 0;

        if (mode == BLUE){
            Pose currentPose = follower.getPose();
            targetDistance = currentPose.distanceFrom(blueGoal);
        }
        else if (mode == RED){
            Pose currentPose = follower.getPose();
            targetDistance = currentPose.distanceFrom(redGoal);
        }

        return targetDistance;
    }

     */
}