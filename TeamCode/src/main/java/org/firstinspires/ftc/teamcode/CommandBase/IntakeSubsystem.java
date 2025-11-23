package org.firstinspires.ftc.teamcode.CommandBase;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.scrims.RobotScrims;


public class IntakeSubsystem extends SubsystemBase {

    RobotScrims robot;

    //Do we even need this:
    public IntakeSubsystem(HardwareMap hardwareMap) {
        robot = new RobotScrims(hardwareMap);
    }

    //Run intake, call from command (emad: figure out how to not keep running forever in command)
    public void IntakeSubsystemRun() {
        robot.intakeRun();
        robot.beltRun();
    }

    public void IntakeSubsystemStop() {
        robot.intake.setPower(0);
        robot.belt.setPower(0);
    }

    public void IntakeSubsystemReverse() {

        ElapsedTime timer = new ElapsedTime();
        double Time1 = 1;
        double Time2 = 0.5; // example times idk

        robot.beltBackRun();
        if (timer.seconds() >= Time1) { // yay it works
            robot.belt.setPower(0);
        }
    }
}
