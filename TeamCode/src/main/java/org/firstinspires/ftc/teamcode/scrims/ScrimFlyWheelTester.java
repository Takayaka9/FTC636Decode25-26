package org.firstinspires.ftc.teamcode.scrims;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PIDFControl_ForVelocity;
/*
Test the flywheel by changing "TargetVelocity" in panels (max 2800, I think). Then press button A
on GAMEPAD 1 (ONE) to move.
CODERS: graph targetV, VelocityL and VelocityR to tune PIDF control.
 */

@Disabled
@Configurable
@TeleOp(name = "FlyWheelTester", group = "TeleOp")
public class ScrimFlyWheelTester extends LinearOpMode {
    RobotScrims robot;
    //PIDF control numbers for flywheel: NEED TO BE TUNED
    //public static double p=0, i=0, d=0, f=0;
    //public static PIDFControl_ForVelocity control = new PIDFControl_ForVelocity(p, i, d, f);
    public static PIDFControl_ForVelocity control = new PIDFControl_ForVelocity(0, 0, 0, 0);
    TelemetryManager telemetryM;
    //change the target velocity in panels/telemetry to test values
    public static double targetVelocity = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        robot = new RobotScrims(hardwareMap);

        waitForStart();

        while(opModeIsActive()){
            telemetryM.update();

            //control.setValues(p, i, d, f);
            telemetryM.debug("Kp", control.Kp);
            telemetryM.debug("Ki", control.Ki);
            telemetryM.debug("Kd", control.Kd);
            telemetryM.debug("Kf", control.Kf);

            if(gamepad1.a){
                double power = control.update(targetVelocity, robot.flyRight.getVelocity());

                robot.flyLeft.setPower(power);
                robot.flyRight.setPower(power);
            }

            telemetryM.debug("TargetVelocity", targetVelocity);
            telemetryM.addData("VelocityR", robot.flyRight.getVelocity());
        }

    }
}
