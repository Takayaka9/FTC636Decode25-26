package org.firstinspires.ftc.teamcode.CommandBase;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.quals.QualsTeleOp;

public class Commands {

    /* RunIntake Command */
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

        @Override
        public void end(boolean interrupted) {
            m_intakesubsystem.IntakeSubsystemStop();
        }

        public boolean isFinished() {
            return true;
        }


    }

    public class RunIntakeReverse extends CommandBase {
        private final IntakeSubsystem m_intakesubsystem;
        public RunIntakeReverse(IntakeSubsystem subsystem) {
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


    public class RunBeltReverse extends CommandBase {
        private final BeltSubsystem m_beltsubsystem;
        public RunBeltReverse(BeltSubsystem subsystem) {
            m_beltsubsystem = subsystem;
            addRequirements(subsystem);
        }
        @Override
        public void initialize(){m_beltsubsystem.beltReverse();}

        @Override
        public void end(boolean interrupted) {
            m_beltsubsystem.beltPassive();
        }
        public boolean isFinished() {return true;}
    }

    public class FlyShoot extends CommandBase {
        private final FlySubsystem m_flysubsystem;
        public FlyShoot(FlySubsystem subsystem) {
            m_flysubsystem = subsystem;
            addRequirements(subsystem);
        }

        @Override
        public void initialize(){m_flysubsystem.flyRun(4500);}

        @Override
        public void end(boolean interrupted) {
            m_flysubsystem.flySTOP();
        }
        public boolean isFinished() {return true;}
    }

}


