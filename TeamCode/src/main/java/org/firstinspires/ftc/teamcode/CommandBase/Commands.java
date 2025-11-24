package org.firstinspires.ftc.teamcode.CommandBase;

import static org.firstinspires.ftc.teamcode.quals.RobotQuals.intakePower;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;

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

    public static class RunIntake extends CommandBase{
        private DcMotorEx intake;
        public RunIntake(DcMotorEx mIntake){
            intake = mIntake;
        }
        @Override
        public void initialize(){
            intake.setPower(intakePower);
        }
        public void end(boolean interrupted){
            intake.setPower(0);
        }
        public boolean isFinished(){
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

    public class FlyShoot extends CommandBase {
        private final FlySubsystem m_flysubsystem;
        public FlyShoot(FlySubsystem subsystem) {
            m_flysubsystem = subsystem;
            addRequirements(subsystem);
        }

        @Override
        public void initialize(){m_flysubsystem.FlyRun();}

        @Override
        public void end(boolean interrupted) {
            m_flysubsystem.FlySTOP();
        }
        public boolean isFinished() {return true;}
    }
}

