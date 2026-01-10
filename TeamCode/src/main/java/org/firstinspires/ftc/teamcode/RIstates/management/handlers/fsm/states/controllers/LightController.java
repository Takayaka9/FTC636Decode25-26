package org.firstinspires.ftc.teamcode.RIstates.management.handlers.fsm.states.controllers;

import org.firstinspires.ftc.teamcode.RIstates.management.handlers.fsm.states.controllers.subsystems.PWMLight;

public class LightController {

    PWMLight light;
    public LightController(PWMLight light){
        this.light = light;
    }

    public void intakeLightingUpdate(int artifactCount) {
        if (artifactCount ==0) {
            light.violet();
        }
        if (artifactCount == 1) {
            light.red();
        }
        if (artifactCount == 2) {
            light.yellow();
        }
        if (artifactCount == 3) {
            light.green();
        }
    }

    public void shooterLightingUpdate(int artifactsShot) {
        if (artifactsShot ==0) {
            light.violet();
        }
        if (artifactsShot == 1) {
            light.red();
        }
        if (artifactsShot == 2) {
            light.yellow();
        }
        if (artifactsShot == 3) {
            light.green();
        }
    }
}
