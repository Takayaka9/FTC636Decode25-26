package org.firstinspires.ftc.teamcode.states;

import static org.firstinspires.ftc.teamcode.states.StatesTeleOp.TurretModes.BLUE;
import static org.firstinspires.ftc.teamcode.states.StatesTeleOp.TurretModes.RED;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.util.InterpLUT;
import com.seattlesolvers.solverslib.geometry.Translation2d;


@Configurable
public class Hood {
    TelemetryManager telemetryM;
    private final InterpLUT lut = new InterpLUT();
    private RobotStates robot;


    /*
    Get Target Distance is a method to retrieve target distance
    inputs: robotPose, (turret) mode
    outputs: targetDistance (also printed to panels)
    call once in opmode before using methods requiring 'targetDistance' then pass into method
     */
    int blueGoalX = 0;
    int blueGoalY = 0;
    int redGoalX = 0;
    int redGoalY = 0;
    Translation2d blueGoal = new Translation2d(blueGoalX, blueGoalY);
    Translation2d redGoal = new Translation2d(redGoalX, redGoalY);

    public int getTargetDistance(Translation2d robotPose, StatesTeleOp.TurretModes mode){
        int targetDistance = 0;

        if (mode == BLUE){
            robotPose.getDistance(blueGoal);
        }
        else if (mode == RED){
            robotPose.getDistance(redGoal);
        }

        return targetDistance;
    }




    /*
    angleHood uses InterpLut to calculate angle of hood based on target distance
    inputs: targetDistance, hardwareMap
    output: panels telemetry and servo position
     */
    double d1 = 0; double p1 = 0; int r1 = 0;
    double d2 = 0; double p2 = 0; int r2 = 0;
    double d3 = 0; double p3 = 0; int r3 = 0;
    double d4 = 0; double p4 = 0; int r4 = 0;
    double d5 = 0; double p5 = 0; int r5 = 0;
    double d6 = 0; double p6 = 0; int r6 = 0;

    public void angleHood(double targetDistance, HardwareMap hardwareMap) {
        robot = new RobotStates(hardwareMap);
        lut.add(d1, p1);
        lut.add(d2, p2);
        lut.add(d3, p3);
        lut.add(d4, p4);
        lut.add(d5, p5);
        lut.add(d6, p6);
        lut.createLUT();

        double angle = lut.get(targetDistance);
        //TODO: figure out what we call this servo
        //robot.servo.setPosition(angle);
        telemetryM.addData("angle", angle);
    }


    /*
    shooterRPM uses InterpLut to calculate shooterRPM based on target distance
    inputs: targetDistance, hardwareMap
    output: panels telemetry
    return: desiredRPM
     */
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
}