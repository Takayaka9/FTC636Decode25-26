package org.firstinspires.ftc.teamcode.RIstates.management.handlers.fsm.states.controllers.subsystems.color;

public interface IsBallPresent {

     enum detectedLocation {
        INTAKE, TURRET, INTAKECLEAR, TURRETCLEAR
    }
    detectedLocation CheckIsBallPresent();
}
