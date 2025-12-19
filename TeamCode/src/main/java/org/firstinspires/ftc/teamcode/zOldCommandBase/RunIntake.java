package org.firstinspires.ftc.teamcode.zOldCommandBase;

import com.seattlesolvers.solverslib.command.CommandBase;

public class RunIntake extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final IntakeSubsystem m_intakesubsystem;

    public RunIntake(IntakeSubsystem subsystem) {
        m_intakesubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_intakesubsystem.IntakeSubsystemRun();
    }

    public void end() {
        m_intakesubsystem.IntakeSubsystemStop();
    }

    public boolean isFinished() {
        return true;
    }


}