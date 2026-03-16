package org.firstinspires.ftc.teamcode.nePremier.robot.systems.servos;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.nePremier.utils.commandUtils.CommandLoop;
import org.firstinspires.ftc.teamcode.nePremier.utils.timers.GenericTime;
import org.firstinspires.ftc.teamcode.nePremier.utils.timers.SolversTiming;
import org.firstinspires.ftc.teamcode.nePremier.utils.servo.CRServoBase;

@Deprecated
public class CrLiftServo extends CRServoBase{
    GenericTime timer;
    int liftTime = 3000;
    public CrLiftServo(CommandLoop maps, HardwareMap hardwareMap) {
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
