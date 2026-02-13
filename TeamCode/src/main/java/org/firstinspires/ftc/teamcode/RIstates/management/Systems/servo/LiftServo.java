package org.firstinspires.ftc.teamcode.RIstates.management.Systems.servo;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.timers.GenericTime;
import org.firstinspires.ftc.teamcode.utils.timers.SolversTiming;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.servo.base.CRServoBase;

@Configurable
public class LiftServo extends CRServoBase{
    GenericTime timer;
    int liftTime = 3000;
    public LiftServo(HardwareMap hardwareMap) {
        super(hardwareMap, "lift");
        timer = new SolversTiming();
        timer.create();
    }

    /// call in update loop
    public void up(){
        timer.setLength(liftTime);
        timer.resetThenStart();
        if (!timer.checkFinished()) {
            run();
        } else if (timer.checkFinished()) {
            stop();
        }
    }

    /// will not stop it
    public void down() {
        reverse();
    }


}
