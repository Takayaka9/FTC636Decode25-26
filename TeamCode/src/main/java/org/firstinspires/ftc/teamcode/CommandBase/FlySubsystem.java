package org.firstinspires.ftc.teamcode.CommandBase;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.scrims.RobotScrims;

public class FlySubsystem extends SubsystemBase {

    RobotScrims robot;
    //still not sure if we need this:
    public FlySubsystem(HardwareMap hardwareMap) {
        robot = new RobotScrims(hardwareMap);
    }
    public void FlyRun() {
        robot.flyRight.setVelocityPIDFCoefficients(1.2, 2.0, 0.001, 0);
        robot.flyLeft.setVelocityPIDFCoefficients(1.2, 2.0, 0.001, 0);
        robot.flyRight.setVelocity(robot.RPMtoVelocity(8000));
        robot.flyLeft.setVelocity(robot.RPMtoVelocity(8000));
    }

    public void FlySTOP() {
        robot.flyRight.setPower(0);
        robot.flyLeft.setPower(0);
    }
}
