package org.firstinspires.ftc.teamcode.zCommandBase;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.zquals.RobotQuals;
import static org.firstinspires.ftc.teamcode.zquals.QualsTeleOp.onRampPush;
import static org.firstinspires.ftc.teamcode.zquals.QualsTeleOp.onRampPassive;
import static org.firstinspires.ftc.teamcode.zquals.QualsTeleOp.offRampPush;
import static org.firstinspires.ftc.teamcode.zquals.QualsTeleOp.offRampPassive;




public class SortSubsystem extends SubsystemBase {
    RobotQuals robot;

    public SortSubsystem(HardwareMap hardwareMap) {
        robot = new RobotQuals(hardwareMap);
    }

    public void setOnRampPush() {
        robot.onRamp.setPosition(onRampPush);
    }
    public void setOnRampPassive() {
        robot.onRamp.setPosition(onRampPassive);
    }
    public void setOffRampPush() {
        robot.onRamp.setPosition(offRampPush);
    }
    public void setOffRampPassive() {
        robot.onRamp.setPosition(offRampPassive);
    }

}
