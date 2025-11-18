package org.firstinspires.ftc.teamcode.scrims.ComandBase;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RunIntakeCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final IntakeSubsystem m_intakesubsystem;
    public RunIntakeCommand(IntakeSubsystem subsystem) {
        m_intakesubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        m_intakesubsystem.IntakeSubsystemRun();
    }

    @Override
    public void end(boolean interrupted) {
        m_intakesubsystem.IntakeSubsystemStop();
    }

    public boolean isFinished() {
        return true;
    }

}
