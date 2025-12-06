package org.firstinspires.ftc.teamcode.CommandBase;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.quals.RobotQuals;

public class FlySubsystem extends SubsystemBase {

    RobotQuals robot;
    //still not sure if we need this:
    public FlySubsystem(HardwareMap hardwareMap) {
        robot = new RobotQuals(hardwareMap);
    }
    public void flyRun(double desiredRPM) {
        robot.shooterPIDF(desiredRPM);
    }

    public void flySTOP() {
        robot.flyRight.setPower(0);
        robot.flyLeft.setPower(0);
    }
}
