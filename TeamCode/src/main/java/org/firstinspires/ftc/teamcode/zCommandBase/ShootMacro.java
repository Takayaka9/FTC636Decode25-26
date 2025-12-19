package org.firstinspires.ftc.teamcode.zCommandBase;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.zquals.RobotQuals;

import static org.firstinspires.ftc.teamcode.zquals.QualsTeleOp.shoot2;
import static org.firstinspires.ftc.teamcode.zquals.QualsTeleOp.shoot3;
import static org.firstinspires.ftc.teamcode.zquals.QualsTeleOp.shoot4;
import static org.firstinspires.ftc.teamcode.zquals.QualsTeleOp.shoot5;
import static org.firstinspires.ftc.teamcode.zquals.QualsTeleOp.shoot6;

public class ShootMacro extends CommandBase {
    RobotQuals robot;
    private final BeltSubsystem m_beltSubsystem;
    private final SortSubsystem m_sortSubsystem;
    private final FlySubsystem m_flySubsystem;
    private final IntakeSubsystem m_intakeSubsystem;

    private final ElapsedTime timer = new ElapsedTime();
    public boolean isFinished;

    public enum ShootingSteps{
        READY, REV_1, SHOOT_1, REV_2, SHOOT_2, REV_3, SHOOT_3, FINISH, END
    }
    private ShootMacro.ShootingSteps shootingSteps = ShootingSteps.READY;
    
    public ShootMacro(BeltSubsystem beltSubsystem, SortSubsystem sortSubsystem, FlySubsystem flySubsystem, IntakeSubsystem intakeSubsystem) {
        m_beltSubsystem = beltSubsystem;
        m_sortSubsystem = sortSubsystem;
        m_flySubsystem = flySubsystem;
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(beltSubsystem);
        addRequirements(sortSubsystem);
        addRequirements(flySubsystem);
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        shootingSteps = ShootMacro.ShootingSteps.READY;
        timer.reset();
    }

    @Override
    public void execute() {
        switch(shootingSteps){
            case READY:
                    isFinished = false;
                    //isShooting = true;
                    //robot.flyRight.setPower(0);
                    //robot.flyLeft.setPower(0);
                    robot.belt.setPower(0);
                    //robot.belt.setTargetPosition(robot.belt.getCurrentPosition());
                    //robot.belt.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    m_intakeSubsystem.IntakeSubsystemStop();
                    m_sortSubsystem.setOffRampPassive();
                    m_sortSubsystem.setOnRampPush();
                    timer.reset();
                    shootingSteps = ShootMacro.ShootingSteps.SHOOT_1;
                break;
            case SHOOT_1:
                robot.belt.setPower(0);
                //m_beltSubsystem.beltPassive();
                //robot.shooterPIDF(velocity);
                //robot.belt.setTargetPosition(robot.belt.getCurrentPosition() + beltIncrement);
                timer.reset();
                shootingSteps = ShootMacro.ShootingSteps.REV_2;
                break;
            case REV_2:
                if(timer.seconds() >= shoot2){
                    robot.belt.setPower(0);
                    //m_beltSubsystem.beltPassive();
                    m_sortSubsystem.setOffRampPush();
                    //robot.shooterPIDF(velocity);
                    timer.reset();
                    shootingSteps = ShootMacro.ShootingSteps.SHOOT_2;
                }
                break;
            case SHOOT_2:
                if(timer.seconds() >= shoot3){
                    m_sortSubsystem.setOffRampPassive();
                    //robot.shooterPIDF(velocity);
                    robot.belt.setPower(0);
                    //m_beltSubsystem.beltPassive();
                    timer.reset();
                    shootingSteps = ShootMacro.ShootingSteps.REV_3;
                }
                break;
            case REV_3:
                if (timer.seconds() >= shoot4) {
                    robot.belt.setPower(0);
                    //m_beltSubsystem.beltPassive();
                    //robot.shooterPIDF(velocity);
                    timer.reset();
                    shootingSteps = ShootMacro.ShootingSteps.SHOOT_3;
                }
                break;
            case SHOOT_3:
                if(timer.seconds() >= shoot5){
                    robot.belt.setPower(0);
                    //m_beltSubsystem.beltPassive();
                    //robot.shooterPIDF(velocity);
                    timer.reset();
                    shootingSteps = ShootMacro.ShootingSteps.FINISH;
                }
                break;
            case FINISH:
                if(timer.seconds()>= shoot6){
                    robot.belt.setPower(0);
                    //m_beltSubsystem.beltPassive();
                    //robot.flyLeft.setPower(0);
                    //robot.flyRight.setPower(0);
                    //isShooting = false;
                    shootingSteps = ShootMacro.ShootingSteps.END;
                }
                break;
            case END:
                isFinished = true;
        }
    }

    public void end() {}
    public boolean isFinished() {return true;}
}
