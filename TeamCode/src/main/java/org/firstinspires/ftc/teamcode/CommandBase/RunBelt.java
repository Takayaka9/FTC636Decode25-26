package org.firstinspires.ftc.teamcode.CommandBase;

import com.arcrobotics.ftclib.command.CommandBase;

public class RunBelt extends CommandBase {
    private final BeltSubsystem m_beltsubsystem;
    public RunBelt(BeltSubsystem subsystem) {
        m_beltsubsystem = subsystem;
        addRequirements(subsystem);
    }
    @Override
    public void initialize(){m_beltsubsystem.beltRun();}

    @Override
    public void end(boolean interrupted) {
        m_beltsubsystem.beltPassive();
    }
    public boolean isFinished() {return true;}
}