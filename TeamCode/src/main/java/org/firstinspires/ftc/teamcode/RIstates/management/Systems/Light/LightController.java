package org.firstinspires.ftc.teamcode.RIstates.management.Systems.Light;

import com.qualcomm.robotcore.hardware.HardwareMap;

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
}
