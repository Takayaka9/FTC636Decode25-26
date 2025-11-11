package org.firstinspires.ftc.teamcode.scrims;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PIDFControl_ForVelocity;

@Configurable
@TeleOp(name = "FlyWheelTester", group = "TeleOp")
public class ScrimFlyWheelTester extends LinearOpMode {
    RobotScrims robot = new RobotScrims(hardwareMap);
    public static PIDFControl_ForVelocity control = new PIDFControl_ForVelocity(0.0, 0.0, 0.0, 0.0);
    TelemetryManager telemetryM;
    public static double targetVelocity = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        waitForStart();

        while(opModeIsActive()){
            telemetryM.update();

            if(gamepad1.a){
                double powerLeft = control.update(targetVelocity, robot.flyLeft.getVelocity());
                double powerRight = control.update(targetVelocity, robot.flyRight.getVelocity());

                robot.flyLeft.setPower(powerLeft);
                robot.flyRight.setPower(powerRight);
            }

            telemetryM.debug("TargetV", targetVelocity);
            telemetryM.debug("VelocityL", robot.flyLeft.getVelocity());
            telemetryM.debug("VelocityR", robot.flyRight.getVelocity());
        }

    }
}
