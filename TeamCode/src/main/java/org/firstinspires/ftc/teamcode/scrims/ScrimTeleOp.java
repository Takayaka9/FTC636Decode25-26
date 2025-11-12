package org.firstinspires.ftc.teamcode.scrims;

import static org.firstinspires.ftc.teamcode.pedroTesting.TeleOPTest.startingPose;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PIDFControl_ForVelocity;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "Scrims TeleOP", group = "TeleOp")
public class ScrimTeleOp extends LinearOpMode {
    private Follower follower;
    private TelemetryManager telemetryM;
    RobotScrims robot;
    PIDFControl_ForVelocity velocityControl = new PIDFControl_ForVelocity(0.0, 0.0, 0.0, 0.0);
    private double targetVelocity;
    public static boolean changedRT = false;
    public static boolean changedRB = false;

    @Override
    public void runOpMode() throws InterruptedException{
        robot = new RobotScrims(hardwareMap);

        waitForStart();

        robot.initialTele();

        if (isStopRequested()) return;

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        follower.startTeleopDrive();

        while(opModeIsActive()){

            telemetryM.update();
            follower.update();

            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true
            );

            if(gamepad2.right_trigger > 0.4 && !changedRT){
                robot.belt.setPower(0.1);
                changedRT = true;
            }
            else if(gamepad2.right_trigger < 0.3){
                robot.belt.setPower(0);
            }

            if(gamepad2.right_bumper && !changedRB){

            }

            telemetryM.debug("amountGreen", robot.colorSensor.green());
            telemetryM.debug("amountRed", robot.colorSensor.red());
            telemetryM.debug("amountBlue", robot.colorSensor.blue());
        }
    }
}
