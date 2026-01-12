package org.firstinspires.ftc.teamcode.RIstates.management.Systems.color;

public interface IsBallPresent {

     enum detectedLocation {
        INTAKE, TURRET, IntakeCLEAR, TurretCLEAR
    }
    detectedLocation CheckIsBallPresent();

     double test();
}
