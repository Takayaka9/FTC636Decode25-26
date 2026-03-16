package org.firstinspires.ftc.teamcode.nePremier.utils.sensors;

@Deprecated
public interface IsBallPresent {

     enum detectedLocation {
        INTAKE, TURRET, IntakeCLEAR, TurretCLEAR
    }
    detectedLocation CheckIsBallPresent();

     double test();
}
