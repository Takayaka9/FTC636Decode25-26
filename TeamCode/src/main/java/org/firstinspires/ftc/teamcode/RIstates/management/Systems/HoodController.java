package org.firstinspires.ftc.teamcode.RIstates.management.Systems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.RIstates.management.Systems.servo.HoodServo;


@Configurable
public class HoodController extends HoodServo {
    private final InterpLUT lut = new InterpLUT();
    public HoodController(HardwareMap hardwareMap){
        super(hardwareMap);
        lut.add(0, p1);
        lut.add(d1, p1);
        lut.add(d2, p2);
        lut.add(d3, p3);
        lut.add(d4, p4);
        lut.add(d5, p5);
        lut.add(d6, p6);
        lut.add(1000, p6);
        lut.createLUT();
    }

    /*
    angleHood uses InterpLut to calculate angle of hoodController based on target distance
    inputs: targetDistance, hardwareMap
    output: panels telemetry and servo position
    !! it is never needed to call this method - it is called by shoot !!
     */
    static double d1 = 36; static double p1 = 0.1;
    static double d2 = 50; double p2 = 0.2;
    static double d3 = 75; static double p3 = 0.3;
    static double d4 = 96; static double p4 = 0.4;
    static double d5 = 108; static double p5 = 0.5;
    static double d6 = 150; static double p6 = 0.6;

    public double angle;
    public void angleHood(double targetDistance) {
        angle = lut.get(targetDistance);
        setPosition(angle);
    }

    public static double passive = 0.5;
    public void passive(){
        setPosition(passive);
    }

    public void increment(boolean positive){
        if(positive){
            setPosition(getPosition() + 0.03);
        }
        else{
            setPosition(getPosition() - 0.03);
        }
    }


    /* graveyard - pre-shooterHandler setup
    shooterRPM uses InterpLut to calculate shooterRPM based on target distance
    inputs: targetDistance, hardwareMap
    output: panels telemetry
    return: desiredRPM

    public int shooterRPM(double targetDistance, HardwareMap hardwareMap){

        robot = new Config(hardwareMap);
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