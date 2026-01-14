package org.firstinspires.ftc.teamcode.RIstates.management.Systems.ballController;

import org.firstinspires.ftc.teamcode.RIstates.management.Systems.color.IntakeDistanceSensor;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.color.IntakeSensor;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.color.IsBallPresent;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.color.TurretSensor;

public class BallCounter {

    IsBallPresent intake;
    IsBallPresent turret;
    public BallCounter(IntakeDistanceSensor intake, TurretSensor turret) {
        this.intake = intake;
        this.turret = turret;
    }

    private static int ballCount = 0;
    public boolean checkBotClear() {
        if (ballCount == 0) {
            return true;
        }
        return false;
    }

    public boolean checkBotFull() {
        if (ballCount == 0) {
            return true;
        }
        return false;
    }

    public int getBallCount() {
        return ballCount;
    }




    boolean changedIntake = false;
    boolean changedTurret = false;

    ///call update method in loop
    public void intakeUpdateBallCount() {
        if (intake.CheckIsBallPresent() == IntakeSensor.detectedLocation.INTAKE) {
            if (!changedIntake) {
                ballCount++;
            }
            changedIntake = true;
        } else if (intake.CheckIsBallPresent() == IntakeSensor.detectedLocation.IntakeCLEAR) {
            changedIntake = false;
        }
    }

    ///call update method in loop
    public void shootUpdateBallCount() {
        if (turret.CheckIsBallPresent() == TurretSensor.detectedLocation.TURRET) {
            if (!changedTurret) {
                ballCount--;
            }
            changedTurret = true;
        } else if (turret.CheckIsBallPresent() == TurretSensor.detectedLocation.TurretCLEAR) {
            changedTurret = false;
        }
    }
}
