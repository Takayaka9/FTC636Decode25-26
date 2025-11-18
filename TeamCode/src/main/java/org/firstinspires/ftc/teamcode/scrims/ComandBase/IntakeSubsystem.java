package org.firstinspires.ftc.teamcode.scrims.ComandBase;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
    }

    public void IntakeSubsystemStop() {
        robot.intake.setPower(0);
    }
}
