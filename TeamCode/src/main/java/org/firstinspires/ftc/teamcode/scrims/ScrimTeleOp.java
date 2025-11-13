package org.firstinspires.ftc.teamcode.scrims;

import static org.firstinspires.ftc.teamcode.pedroTesting.TeleOPTest.startingPose;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PIDFControl_ForVelocity;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "Scrims TeleOP", group = "TeleOp")
public class ScrimTeleOp extends LinearOpMode {
    Follower follower;
    TelemetryManager telemetryM;
    RobotScrims robot;

    //Velocities for shooters
    //TODO: test values
    public static int velocityClose = 1000;
    public static int velocityFar = 2000;


    //debouncers: prevents the code from repeating itself until the button is released and pressed again
    public static boolean changedRB = false;
    public static boolean changed1A = false;
    public static boolean changed2A = false;
    public static boolean changed2B = false;


    @Override
    public void runOpMode() throws InterruptedException{
        robot = new RobotScrims(hardwareMap);

        waitForStart();

        robot.initialTele();

        if (isStopRequested()) return;

        //Pedro follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        follower.startTeleopDrive();
        robot.limelight3A.start();

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
            if(gamepad2.a && !changed2A){
                robot.shoot(velocityClose);
                changed2A = true;
            }
            else if(!gamepad2.a){
                changed2A = false;
            }

            //shoot with more velocity (far pos)
            if(gamepad2.b && !changed2B){
                robot.shoot(velocityFar);
                changed2B = true;
            }
            else if(!gamepad2.b){
                changed2B = false;
            }

            //to be coded: change to sort mode
            if(gamepad1.a && !changed1A){

            }

            telemetryM.addData("amountGreen", robot.colorSensor.green());
            telemetryM.addData("amountRed", robot.colorSensor.red());
            telemetryM.addData("amountBlue", robot.colorSensor.blue());
        }
    }
}
