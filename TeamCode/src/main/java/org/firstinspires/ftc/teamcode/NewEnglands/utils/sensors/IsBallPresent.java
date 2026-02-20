package org.firstinspires.ftc.teamcode.NewEnglands.utils.sensors;

public interface IsBallPresent {

     enum detectedLocation {
        INTAKE, TURRET, IntakeCLEAR, TurretCLEAR
    }
    detectedLocation CheckIsBallPresent();

     double test();
}
