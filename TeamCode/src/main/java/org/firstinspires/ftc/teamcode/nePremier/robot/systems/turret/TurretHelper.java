package org.firstinspires.ftc.teamcode.nePremier.robot.systems.turret;

import static org.firstinspires.ftc.teamcode.nePremier.robot.systems.turret.EmadTurret.EmadTurretConstants.ticksPerRev;

@Deprecated
public class TurretHelper {
    public static double clamp(double radians) {
        radians = Math.max(radians, -Math.PI/2);
        radians = Math.min(radians, Math.PI/2);
        return radians;
    }
    public static double getFracRads(double radians) {
        return (radians*((ticksPerRev*5.1)/(Math.PI*2)));
    }

}
