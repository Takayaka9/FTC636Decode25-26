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
    public static boolean changedRB = false;
    public static boolean changed1A = false;
    public static boolean changedA = false;
    public static boolean changedB = false;
    public static int velocityClose = 1000;
    public static int velocityFar = 2000;
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

            //drivetrain...I think it works...
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true
            );

            //manually move the belt to move the balls
            if(gamepad2.right_trigger != 0){
                robot.belt.setPower(0.1);
            }
            else{
                robot.belt.setPower(0);
            }

            //activate intake
            if(gamepad2.left_trigger != 0){
                robot.intake.setPower(0.5);
            }
            else{
                robot.intake.setPower(0);
            }

            //shoot with less velocity (close pos)
            if(gamepad2.a && !changedA){
                robot.shoot(velocityClose);
                changedA = true;
            }
            else if(!gamepad2.a){
                changedA = false;
            }

            //shoot with more velocity (far pos)
            if(gamepad2.b && !changedB){
                robot.shoot(velocityFar);
                changedB = true;
            }
            else if(!gamepad2.b){
                changedB = false;
            }

            //to be coded: change to sort mode
            if(gamepad1.a && !changed1A){
            }

            telemetryM.debug("amountGreen", robot.colorSensor.green());
            telemetryM.debug("amountRed", robot.colorSensor.red());
            telemetryM.debug("amountBlue", robot.colorSensor.blue());
        }
    }
}
