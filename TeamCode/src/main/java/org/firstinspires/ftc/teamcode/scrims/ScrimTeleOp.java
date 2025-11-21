package org.firstinspires.ftc.teamcode.scrims;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    public static int velocityClose = 3000;
    public static int velocityFar = 5000;
    public static double beltOn = 0.5;
    public static double intakeOn = 0.7;
    public static int beltTargetPosition = 0;
    public static int beltIncrement = 1;

    //debouncers: prevents the code from repeating itself until the button is released and pressed again
    public static boolean intakeToggle = false;
    public static boolean changedRB = false;
    public static boolean changed1A = false;
    public static boolean changed2A = false;
    public static boolean changed2B = false;
    public static boolean changed2Y = false;
    public static boolean isSorting = false;
    public static double shootP, shootI, shootD, shootF;
    ElapsedTime sortTime = new ElapsedTime();
    public static double sort1 = 0.3;
    public static double sort2 = 1;

    @Override
    public void runOpMode() throws InterruptedException{
        robot = new RobotScrims(hardwareMap);
        //robot.belt.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        waitForStart();

        robot.initialTele();

        if (isStopRequested()) return;

        //Pedro follower
        follower = Constants.createFollower(hardwareMap);
        //follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        follower.startTeleopDrive();
        //robot.limelight3A.start();

        while(opModeIsActive()){

            telemetryM.update();
            follower.update();



            //drivetrain...I think it works...
            follower.setTeleOpDrive(
                    gamepad1.left_stick_x,
                    -gamepad1.left_stick_y,
                    -gamepad1.right_stick_x,
                    true
            );

            //move the belt and intake to move the balls
            if(!isSorting){
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
                else if(gamepad2.x){
                    robot.belt.setPower(-beltOn);
                    robot.intake.setPower(-intakeOn);
                }
            }

            /*
            //shoot with less velocity (close pos)
            if(gamepad2.a){
                robot.flyRight.setVelocity(robot.RPMtoVelocity(velocityFar));
                //robot.flyRight.setPower(1);
                robot.flyLeft.setVelocity(robot.RPMtoVelocity(velocityFar));
                //changed2A = true;
            }
            else if(!gamepad2.a){
                robot.flyRight.setVelocity(robot.RPMtoVelocity(0));
                robot.flyLeft.setVelocity(robot.RPMtoVelocity(0));
            }

             */

            //shoot
            if(gamepad2.b){
                //robot.flyRight.setVelocityPIDFCoefficients(shootP, shootI, shootD, shootF);
                //robot.flyLeft.setVelocityPIDFCoefficients(shootP, shootI, shootD, shootF);
                //robot.flyRight.setVelocity(robot.RPMtoVelocity(velocityClose));
                //robot.flyLeft.setVelocity(robot.RPMtoVelocity(velocityClose));
                robot.flyRight.setPower(1);
                robot.flyLeft.setPower(1);
                //changed2B = true;
            }
            else if(!gamepad2.b){
                //changed2B = false;
                //robot.flyRight.setVelocity(robot.RPMtoVelocity(0));
                //robot.flyLeft.setVelocity(robot.RPMtoVelocity(0));
                robot.flyRight.setPower(0);
                robot.flyLeft.setPower(0);
            }
            /*
            if(gamepad2.y){
                robot.pushOff();

            }

             */

            /*
            if(gamepad2.y && !changed2Y){
                beltTargetPosition += beltIncrement;
                robot.belt.setTargetPosition(beltTargetPosition);
                robot.belt.setPower(beltOn);
            }
             */

            /*
            switch(sortSteps){
                case READY:
                    if(gamepad2.y && !changed2Y){
                        changed2Y= true;
                        isSorting = true;
                        robot.belt.setPower(0);
                        robot.intake.setPower(0);
                        sortTime.reset();
                        sortSteps = SortSteps.PUSH;
                    }
                    else if(!gamepad2.y){
                        changed2Y = false;
                    }
                    break;
                case PUSH:
                    robot.pushOff();
                    if(!gamepad2.y){
                        changed2Y = false;
                    }
                    if(sortTime.seconds() > sort1){
                        sortSteps = SortSteps.UP;
                        sortTime.reset();
                    }
                    break;
                case UP:
                    robot.belt.setPower(beltOn);
                    if(sortTime.seconds() > sort2){
                        robot.belt.setPower(0);
                        sortSteps = SortSteps.ON;
                    }
                    break;
                case ON:
                    robot.pushOn();
                    isSorting = false;
                    break;
            }

             */

            telemetryM.debug("flywheel close", velocityClose);
            telemetryM.debug("flywheel far", velocityFar);
            telemetryM.addData("velocity left", robot.flyLeft.getVelocity());
            telemetryM.addData("velocity right", robot.flyRight.getVelocity());
            telemetryM.debug("belt power", beltOn);
            telemetryM.debug("intake power", intakeOn);
            telemetryM.addData("intake toggle", intakeToggle);
            telemetryM.addData("belt target", beltTargetPosition);
            telemetryM.debug("belt increment", beltIncrement);
        }
    }

    public enum SortSteps{
        READY, PUSH, UP, ON
    }

    public SortSteps sortSteps;

}
