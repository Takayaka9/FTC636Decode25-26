package org.firstinspires.ftc.teamcode.scrims.CommandBase;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Commands {

    /* RunIntake Command */
    public class RunIntakeCommand extends CommandBase {
        @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
        private final IntakeSubsystem m_intakesubsystem;

        public RunIntakeCommand(IntakeSubsystem subsystem) {
            m_intakesubsystem = subsystem;
            addRequirements(subsystem);
        }

        @Override
        public void initialize() {
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

    public class RunBeltReverse extends CommandBase {
        private final IntakeSubsystem m_intakesubsystem;
        public RunBeltReverse(IntakeSubsystem subsystem) {
            m_intakesubsystem = subsystem;
            addRequirements(subsystem);
        }
        @Override
        public void initialize(){m_intakesubsystem.IntakeSubsystemReverse();}

        @Override
        public void end(boolean interrupted) {
            m_intakesubsystem.IntakeSubsystemStop();
        }
        public boolean isFinished() {return true;}
    }
}

