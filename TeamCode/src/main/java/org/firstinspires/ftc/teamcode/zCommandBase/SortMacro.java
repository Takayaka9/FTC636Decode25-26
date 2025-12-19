package org.firstinspires.ftc.teamcode.zCommandBase;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import static org.firstinspires.ftc.teamcode.zquals.QualsTeleOp.sort1;
import static org.firstinspires.ftc.teamcode.zquals.QualsTeleOp.sort2;
import static org.firstinspires.ftc.teamcode.zquals.QualsTeleOp.sort3;
import static org.firstinspires.ftc.teamcode.zquals.QualsTeleOp.sort4;
import static org.firstinspires.ftc.teamcode.zquals.QualsTeleOp.sort5;

import org.firstinspires.ftc.teamcode.zquals.RobotQuals;

public class SortMacro extends CommandBase {

    private final BeltSubsystem m_beltSubsystem;
    private final SortSubsystem m_sortSubsystem;
    private final IntakeSubsystem m_intakeSubsystem;

    RobotQuals robot;

    private final ElapsedTime timer = new ElapsedTime();

    private SortingSteps sortingSteps = SortingSteps.READY;


    public enum SortingSteps{
        READY, PUSHOFF, PUSHBACK, UP, PUSHON, BACKOFF, END
    }

    public SortMacro(BeltSubsystem beltSubsystem, SortSubsystem sortSubsystem, IntakeSubsystem intakeSubsystem) {
        m_beltSubsystem = beltSubsystem;
        m_sortSubsystem = sortSubsystem;
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(beltSubsystem);
        addRequirements(sortSubsystem);
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        sortingSteps = SortingSteps.READY;
        timer.reset();
    }

    @Override
    public void execute() {
        switch (sortingSteps) {
            case READY:
                robot.belt.setPower(0);
                //m_beltSubsystem.beltPassive();
                //robot.belt.setTargetPosition(robot.belt.getCurrentPosition());
                //robot.belt.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                m_intakeSubsystem.IntakeSubsystemStop();
                m_sortSubsystem.setOffRampPassive();
                m_sortSubsystem.setOnRampPassive();
                timer.reset();
                sortingSteps = SortingSteps.PUSHOFF;
                break;
            case PUSHOFF:
                m_sortSubsystem.setOnRampPush();
                if(timer.seconds() >= sort1){
                    timer.reset();
                    sortingSteps = SortingSteps.PUSHBACK;
                }
                break;
            case PUSHBACK:
                if(timer.seconds() >= sort2){
                    m_sortSubsystem.setOnRampPassive();
                    timer.reset();
                    sortingSteps = SortingSteps.UP;
                }
                break;
            case UP:
                if(timer.seconds() >= sort3){
                    m_beltSubsystem.beltRun();
                    //robot.belt.setTargetPosition(robot.belt.getCurrentPosition() + beltIncrement);
                    timer.reset();
                    sortingSteps = SortingSteps.PUSHON;
                }
                break;
            case PUSHON:
                if(timer.seconds() >= sort4){
                    robot.belt.setPower(0);
                    //m_beltSubsystem.beltPassive();
                    m_sortSubsystem.setOffRampPush();
                    timer.reset();
                    sortingSteps = SortingSteps.BACKOFF;
                }
                break;
            case BACKOFF:
                if(timer.seconds() >= sort5){
                    m_sortSubsystem.setOffRampPassive();
                    //robot.belt.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    timer.reset();
                    sortingSteps = SortingSteps.END;
                }
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {}
    public boolean isFinished() {return true;}
}
