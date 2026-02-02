package org.firstinspires.ftc.teamcode.RIstates.management.Systems.servo;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Controller;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Intake;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Shooter;

@Configurable
public class IntakeController extends Intake implements Controller {
    private final Shooter shooter;
    private static int toleranceHIGH = 300; /// in TPS (to test)
    private static int toleranceLOW = -300;  /// In TPS

    public IntakeController(Shooter shooter, HardwareMap hardwareMap, String name) {
        super(hardwareMap, name);
        //super(hardwareMap, name);
        this.shooter = shooter;
    }

    private int max = 0;
    private int min = 0;
    public boolean checkShooterReady() {
        max = shooter.averageVelocity() + toleranceHIGH;
        min = shooter.averageVelocity() + toleranceLOW;
        if (shooter.averageVelocity() < max && shooter.averageVelocity() > min) {
            return true;
        }
        return false;
    }

    public void init() {
        run();
    }

    public void update() {
        if (checkShooterReady()) {
            run();
        } else {
            stop();
        }
    }
    public void end() {
        stop();
    }
    public errors updateError() {
        return errors.RUNNING;
    }



}
