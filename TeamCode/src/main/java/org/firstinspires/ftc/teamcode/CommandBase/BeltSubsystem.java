package org.firstinspires.ftc.teamcode.CommandBase;

import static org.firstinspires.ftc.teamcode.quals.QualsTeleOp.offRampPassive;
import static org.firstinspires.ftc.teamcode.quals.QualsTeleOp.offRampPush;
import static org.firstinspires.ftc.teamcode.quals.QualsTeleOp.onRampPassive;
import static org.firstinspires.ftc.teamcode.quals.QualsTeleOp.onRampPush;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.quals.RobotQuals;

public class BeltSubsystem extends SubsystemBase {
        RobotQuals robot;

        public BeltSubsystem(HardwareMap hardwareMap) {
            robot = new RobotQuals(hardwareMap);
        }

        public void beltRun() {
            robot.belt.setPower(1);
        }
       public void beltPassive() {
            robot.belt.setPower(0);
        }
        public void beltReverse() {
            robot.belt.setPower(1);
        }
}
