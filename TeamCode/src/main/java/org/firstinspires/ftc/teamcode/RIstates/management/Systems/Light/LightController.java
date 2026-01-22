package org.firstinspires.ftc.teamcode.RIstates.management.Systems.Light;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RIstates.management.fsm.FSM;

public class LightController extends PWMLight {
    public LightController(HardwareMap hardwareMap) {
        super(hardwareMap);

    }
    public void intakeLightingUpdate(int artifactCount) {
        if (artifactCount ==0) {
            violet();
        }
        if (artifactCount == 1) {
            red();
        }
        if (artifactCount == 2) {
            yellow();
        }
        if (artifactCount == 3) {
            green();
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
            else if (alliance == 1) {
                red();
            }
            else if (alliance == 2) {
                blue();
            }
        }


    }
}
