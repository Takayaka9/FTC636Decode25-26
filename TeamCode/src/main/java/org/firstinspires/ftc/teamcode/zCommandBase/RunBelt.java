package org.firstinspires.ftc.teamcode.zCommandBase;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.zquals.RobotQuals;

public class RunBelt extends CommandBase {
    private final BeltSubsystem m_beltsubsystem;
    RobotQuals robot;
    public RunBelt(BeltSubsystem subsystem) {
        m_beltsubsystem = subsystem;
        addRequirements(subsystem);
    }
    @Override
    public void initialize(){m_beltsubsystem.beltRun();}

    @Override
    public void end(boolean interrupted) {
        robot.belt.setPower(0);
    }
    public boolean isFinished() {return true;}
}