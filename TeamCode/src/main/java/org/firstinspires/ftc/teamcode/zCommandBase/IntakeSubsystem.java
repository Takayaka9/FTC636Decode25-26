package org.firstinspires.ftc.teamcode.zCommandBase;

import static org.firstinspires.ftc.teamcode.zquals.QualsTeleOp.intakeOn;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.zquals.RobotQuals;
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
