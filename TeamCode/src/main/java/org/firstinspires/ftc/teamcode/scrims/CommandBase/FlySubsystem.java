package org.firstinspires.ftc.teamcode.scrims.CommandBase;

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
        robot.flyRight.setPower(1);
        robot.flyLeft.setPower(1);
    }

    public void FlySTOP() {
        robot.flyRight.setPower(0);
        robot.flyLeft.setPower(0);
    }
}
