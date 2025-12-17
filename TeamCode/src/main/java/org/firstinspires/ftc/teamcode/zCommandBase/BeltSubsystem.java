package org.firstinspires.ftc.teamcode.zCommandBase;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.zquals.RobotQuals;

public class BeltSubsystem extends SubsystemBase {
        RobotQuals robot;

        public BeltSubsystem(HardwareMap hardwareMap) {
            robot = new RobotQuals(hardwareMap);
        }

        public void beltRun() {
            robot.belt.setPower(1);
        }
       //public void beltPassive() {
            //robot.belt.setPower(0);
        //}
        public void beltReverse() {
            robot.belt.setPower(-1);
        }
}
