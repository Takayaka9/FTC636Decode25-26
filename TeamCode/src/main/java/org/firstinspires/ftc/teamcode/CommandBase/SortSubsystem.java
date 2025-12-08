package org.firstinspires.ftc.teamcode.CommandBase;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.quals.RobotQuals;
import static org.firstinspires.ftc.teamcode.quals.QualsTeleOp.onRampPush;
import static org.firstinspires.ftc.teamcode.quals.QualsTeleOp.onRampPassive;
import static org.firstinspires.ftc.teamcode.quals.QualsTeleOp.offRampPush;
import static org.firstinspires.ftc.teamcode.quals.QualsTeleOp.offRampPassive;




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
