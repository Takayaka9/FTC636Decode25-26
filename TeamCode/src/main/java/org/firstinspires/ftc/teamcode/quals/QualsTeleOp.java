package org.firstinspires.ftc.teamcode.quals;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Sorting.SortLogic;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Configurable
@TeleOp(name = "Quals TeleOp", group = "TeleOp")
public class QualsTeleOp extends LinearOpMode {
    Follower follower;
    TelemetryManager telemetryM;
    RobotQuals robot;
    Limelight3A limelight;

    IMU imu;
    public static Pose startingPose;

    //Velocities for shooters
    //TODO: test values
    public int Velocity = 8000;
    public static double beltOn = 0.5;
    public static double intakeOn = 0.7;
    public static int beltTargetPosition = 0;
    public static int beltIncrement = 1;
    public static double beltReverse = -0.4;
    public static double flyReverse = -0.3;
    //debouncers: prevents the code from repeating itself until the button is released and pressed again
    public static boolean intakeToggle = false;
    public static boolean changedRB = false;
    public static boolean changed1A = false;
    public static boolean changed2A = false;
    public static boolean changed2B = false;
    public static boolean changed2Y = false;
    public static boolean isSorting = false;
    public static boolean isShooting = false;

    /* old pid constants
    public static double shootP = 1.2, shootI = 2.0, shootD = 0.001, shootF = 0;
     */
    ElapsedTime sortTime = new ElapsedTime();
    public static double sort1 = 0.3;
    public static double sort2 = 1;

    public SortSteps sortSteps;
    public enum SortSteps{
        READY, PUSH, UP, ON
    }


    /*
    Object array which holds ball positions [0] to [2] and sort-cycles [3] need to move requiredArtifact to pos [1]
     */
    SortLogic sortLogic = new SortLogic();
    Object[] sortData = new String[4];

    @Override
    public void runOpMode() throws InterruptedException{
        robot = new RobotQuals(hardwareMap);
        robot.belt.setTargetPosition(robot.belt.getCurrentPosition());
        robot.belt.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        waitForStart();

        //robot.initialTele();

        if (isStopRequested()) return;

        //Pedro follower
        follower = Constants.createFollower(hardwareMap);
        //follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        follower.startTeleopDrive();
        //limelight.start();

        while(opModeIsActive()){
            telemetryM.update();
            follower.update();
            follower.setTeleOpDrive(
                    gamepad1.left_stick_x*0.75,
                    -gamepad1.left_stick_y*0.75,
                    -gamepad1.right_stick_x*0.4,
                    true
            );


            //limelight localization
            /*
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                if (result.isValid()) {
                    Pose3D botpose = result.getBotpose();
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("Botpose", botpose.toString());
                }
            }

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
            result = limelight.getLatestResult();
            if (result != null) {
                if (result.isValid()) {
                    Pose3D botpose = result.getBotpose_MT2();
                }
            }

             */


            //belt reverse control logic
            //also needs to go to RobotQuals
            if(!isSorting){
                if(gamepad2.right_bumper){
                    robot.belt.setPower(beltOn);
                    robot.intake.setPower(intakeOn);
                }
                else if(gamepad2.x){
                    robot.belt.setPower(beltReverse);
                    robot.flyRight.setPower(flyReverse);
                    robot.flyLeft.setPower(flyReverse);
                    robot.intake.setPower(-0.7);
                }
                else if(gamepad2.left_bumper){
                    robot.belt.setPower(beltReverse);
                    robot.flyRight.setPower(flyReverse);
                    robot.flyLeft.setPower(flyReverse);
                    robot.intake.setPower(0);
                }
                else{
                    robot.belt.setPower(0);
                    robot.intake.setPower(0);
                    robot.flyRight.setPower(0);
                    robot.flyLeft.setPower(0);
                }
            }


            //physical sort method
            //maybe should move this to RobotQuals at some point for organization (will be used in auto)
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
            //New shooting by emad:
            //Still incomplete needs a lot more work - emad
            //Shoot green
            if(gamepad2.b){
                isShooting = true;
                robot.shooterPIDF(Velocity);
                //need to have an if statement:
                sortData = sortLogic.updateShotArtifact(sortData);
            } else if(!gamepad2.b){
                //changed2B = false;
                isShooting = false;
                robot.flyRight.setPower(0);
                robot.flyLeft.setPower(0);
            }




            //shoot purple
/*
            //Shoot any
            if(gamepad2.a){
                isShooting = true;
                robot.shooterPIDF(Velocity);
                //need to have an if statement:
                sortData = sortLogic.updateShotArtifact(sortData);

            } else if(!gamepad2.a){
                //changed2B = false;
                isShooting = false;
                robot.flyRight.setPower(0);
                robot.flyLeft.setPower(0);
            }

             */


            /*
            //OLD: move the belt and intake to move the balls
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

             */

            /* legacy shooting input and motor activation
            if(gamepad2.b){
                isShooting = true;
                robot.flyRight.setVelocityPIDFCoefficients(shootP, shootI, shootD, shootF);
                robot.flyLeft.setVelocityPIDFCoefficients(shootP, shootI, shootD, shootF);
                robot.flyRight.setVelocity(robot.RPMtoVelocity(velocityClose));
                robot.flyLeft.setVelocity(robot.RPMtoVelocity(velocityClose));
                //changed2B = true;
            }
            else if(!gamepad2.b){
                //changed2B = false;
                isShooting = false;
                robot.flyRight.setVelocity(robot.RPMtoVelocity(0));
                robot.flyLeft.setVelocity(robot.RPMtoVelocity(0));
                robot.flyRight.setPower(0);
                robot.flyLeft.setPower(0);
            }
            //shoot with less velocity (close pos)
            //gamepad2.a is used to shoot green now
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

            /*
            Don't know what this is - emad
            if(gamepad2.y){
                robot.pushOff();

            }

             */

            /*
            belt pid attempt
            if(gamepad2.y && !changed2Y){
                beltTargetPosition += beltIncrement;
                robot.belt.setTargetPosition(beltTargetPosition);
                robot.belt.setPower(beltOn);
            }
             */

            telemetryM.debug("Auto Velocity", Velocity);
            telemetryM.addData("velocity left", robot.flyLeft.getVelocity());
            telemetryM.addData("velocity right", robot.flyRight.getVelocity());
            telemetryM.debug("belt power", beltOn);
            telemetryM.debug("intake power", intakeOn);
            telemetryM.addData("intake toggle", intakeToggle);
            telemetryM.addData("belt target", beltTargetPosition);
            telemetryM.debug("belt increment", beltIncrement);
            telemetryM.addData("sortData", sortData);
            telemetryM.addData("PIDF+FF Output", robot.shooterOutput);
        }
    }

    public enum DetectedColor{
        GREEN,
        PURPLE,
        UNKNOWN
    }

    public DetectedColor getDetectedColor(TelemetryManager telemetryManager){
        NormalizedRGBA colors = robot.colorSensor.getNormalizedColors();

        float normRed, normGreen, normBlue;

        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;

        //TODO: calibrate the thresholds and add if statements

        telemetryManager.addData("Red", normRed);
        telemetryManager.addData("Green", normGreen);
        telemetryManager.addData("Blue", normBlue);

        return DetectedColor.UNKNOWN;
    }

}
