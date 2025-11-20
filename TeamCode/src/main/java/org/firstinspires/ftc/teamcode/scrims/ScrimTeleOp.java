package org.firstinspires.ftc.teamcode.scrims;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "Scrims TeleOP", group = "TeleOp")
public class ScrimTeleOp extends LinearOpMode {
    Follower follower;
    TelemetryManager telemetryM;
    RobotScrims robot;
    public static Pose startingPose;

    //Velocities for shooters
    //TODO: test values
    public static int velocityClose = 3500;
    public static int velocityFar = 5000;
    public static double beltOn = 0.5;
    public static int intakeOn = 1;
    public static int beltTargetPosition = 0;
    public static int beltIncrement = 1;

    //debouncers: prevents the code from repeating itself until the button is released and pressed again
    public static boolean intakeToggle = false;
    public static boolean changedRB = false;
    public static boolean changed1A = false;
    public static boolean changed2A = false;
    public static boolean changed2B = false;

    public static boolean changed2Y = false;


    @Override
    public void runOpMode() throws InterruptedException{
        robot = new RobotScrims(hardwareMap);

        waitForStart();

        robot.initialTele();

        if (isStopRequested()) return;

        //Pedro follower
        follower = Constants.createFollower(hardwareMap);
        //follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
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

            //move the belt and intake to move the balls
            if(gamepad2.right_bumper && !intakeToggle && !changedRB){
                robot.belt.setPower(beltOn);
                robot.intake.setPower(intakeOn);
                intakeToggle = true;
                changedRB = true;
            }
            else if(gamepad2.right_bumper && intakeToggle && !changedRB){
                robot.belt.setPower(0);
                robot.intake.setPower(0);
                intakeToggle = false;
                changedRB = true;
            }
            else if(!gamepad2.right_bumper){
                changedRB = false;
            }

            //shoot with less velocity (close pos)
            if(gamepad2.a && !changed2A){
                robot.shoot(velocityClose);
                changed2A = true;
            }
            else if(!gamepad2.a){
                changed2A = false;
                robot.shoot(0);
            }

            //shoot with more velocity (far pos)
            if(gamepad2.b && !changed2B){
                robot.shoot(velocityFar);
                changed2B = true;
            }
            else if(!gamepad2.b){
                changed2B = false;
                robot.shoot(0);
            }

            //Sort change
            if(gamepad2.y && !changed2Y){
                beltTargetPosition += beltIncrement;
                robot.belt.setTargetPosition(beltTargetPosition);
                robot.belt.setPower(beltOn);
            }

            telemetryM.debug("flywheel close", velocityClose);
            telemetryM.debug("flywheel far", velocityFar);
            telemetryM.debug("belt power", beltOn);
            telemetryM.debug("intake power", intakeOn);
            telemetryM.addData("intake toggle", intakeToggle);
            telemetryM.addData("belt target", beltTargetPosition);
            telemetryM.debug("belt increment", beltIncrement);
        }
    }
}
