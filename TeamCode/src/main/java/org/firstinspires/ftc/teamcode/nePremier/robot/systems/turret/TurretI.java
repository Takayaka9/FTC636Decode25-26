package org.firstinspires.ftc.teamcode.nePremier.robot.systems.turret;

public interface TurretI {
    void trackGoal();
    void turnTurret(double tPosition);
    void resetEncoder();
    double turretPosition();
}
