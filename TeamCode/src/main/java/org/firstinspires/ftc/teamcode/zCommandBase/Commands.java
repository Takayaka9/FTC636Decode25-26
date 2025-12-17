package org.firstinspires.ftc.teamcode.zCommandBase;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.zquals.RobotQuals;

public class Commands {

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



    public class RunBeltReverse extends CommandBase {
        private final BeltSubsystem m_beltsubsystem;
        RobotQuals robot;
        public RunBeltReverse(BeltSubsystem subsystem) {
            m_beltsubsystem = subsystem;
            addRequirements(subsystem);
        }
        @Override
        public void initialize(){m_beltsubsystem.beltReverse();}

        @Override
        public void end(boolean interrupted) {
            robot.belt.setPower(0);
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


