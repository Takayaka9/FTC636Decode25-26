package org.firstinspires.ftc.teamcode.CommandBase;

import static org.firstinspires.ftc.teamcode.quals.QualsTeleOp.intakeOn;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.quals.RobotQuals;
public class IntakeSubsystem extends SubsystemBase {

    RobotQuals robot;
    public IntakeSubsystem(HardwareMap hardwareMap) {
        robot = new RobotQuals(hardwareMap);
    }

    //Run intake, call from command (emad: figure out how to not keep running forever in command)
    public void IntakeSubsystemRun() {
        robot.intake.setPower(intakeOn);
    }

    public void IntakeSubsystemStop() {
        robot.intake.setPower(0);
    }

    public void IntakeSubsystemReverse() {
        robot.intake.setPower(-intakeOn);
    }
}
