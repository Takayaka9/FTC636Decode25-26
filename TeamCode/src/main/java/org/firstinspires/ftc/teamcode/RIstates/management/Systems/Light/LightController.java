package org.firstinspires.ftc.teamcode.RIstates.management.Systems.Light;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RIstates.management.Systems.color.IntakeDistanceSensor;
import org.firstinspires.ftc.teamcode.RIstates.management.fsm.FSM;

public class LightController extends PWMLight {
    IntakeDistanceSensor distanceSensor;
    public LightController(HardwareMap hardwareMap, IntakeDistanceSensor distance) {
        super(hardwareMap);
        distanceSensor = distance;
    }

    boolean green = false;
    public void update(boolean limeLight, FSM.StateName currentState, int alliance, int averageVelocity, int targetVelocity) {
        if (limeLight) {
            if (!green) {
                green();
                green = true;
            } else if (green) {
                white();
                green = false;
            }
        }
        if (currentState == FSM.StateName.Norm) {
            white();
        }
        if (currentState == FSM.StateName.Shoot) {
            getColor(averageVelocity, targetVelocity);
        }
        if (currentState == FSM.StateName.Intake) {
            violet();
        }
        if (currentState == FSM.StateName.Backout) {
            violet();
        }
        if (limeLight) {
            if (!green) {
                green();
                green = true;
            } else {
                white();
                green = false;
            }
        }
        if (currentState == FSM.StateName.AllianceSelect) {
            if (alliance == 0) {
                white();
            }
            if (alliance == 1) {
                blue();
            }
            if (alliance == 2) {
                red();
            }
        }

    }

    public void getColor(int averageVelocity, int targetVelocity) {
        if (averageVelocity < (targetVelocity-100)) {
            red();
        } else if (averageVelocity > (targetVelocity+200)) {
            indigo();
        } else if (averageVelocity < (targetVelocity-100) && averageVelocity > (targetVelocity+200)) {
            green();
        }
    }



    /*
    ElapsedTime timer = new ElapsedTime();
    boolean found = false;
    public boolean checkFull(){
        boolean detected = distanceSensor.checkDistanceSensor(20);
        if(detected){
            if(!found){
                timer.reset();
                found = true;
            }
            if(timer.seconds() > 1){
                return true;
            }
        }
        else{
            found = false;
        }
        return false;
    }
    public void intakeLightingUpdate(int artifactCount) {
        if (checkFull()) {
            green();
        }
        else{
            red();
        }
    }

    public void shooterLightingUpdate(int artifactsShot) {
        if (artifactsShot ==0) {
            violet();
        }
        if (artifactsShot == 1) {
            red();
        }
        if (artifactsShot == 2) {
            yellow();
        }
        if (artifactsShot == 3) {
            green();
        }
    }
     */
}
