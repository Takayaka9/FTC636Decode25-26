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

    public void update(boolean limeLight, FSM.StateName currentState, int alliance) {
        if (limeLight) {
            green();
        }
        else if (currentState == FSM.StateName.Norm) {
            white();
        }
        else if (currentState == FSM.StateName.Shoot) {
            violet();
        }
        else if (currentState == FSM.StateName.Intake) {
            blue();
        }
        else if (currentState == FSM.StateName.Backout) {
            red();
        }
        else if (currentState == FSM.StateName.AllianceSelect) {
            if (alliance == 0) {
                yellow();
            }
            else if (alliance == 2) {
                red();
            }
            else if (alliance == 1) {
                blue();
            }
        }


    }
}
