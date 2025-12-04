package org.firstinspires.ftc.teamcode.quals;

import org.firstinspires.ftc.teamcode.Sorting.SortLogic;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
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

    //IMU imu;
    public static Pose startingPose;

    //Velocities for shooters
    //TODO: test values
    public int velocity = 4500;
    public static double beltOn = 1;
    public static double intakeOn = 1;
    public static int beltTargetPosition = 0;
    public static int beltIncrement = 100;
    public static double beltReverse = -0.4;
    //public static double flyReverse = -0.3;
    //debouncers: prevents the code from repeating itself until the button is released and pressed again
    public static boolean intakeToggle = false;
    public static boolean changedRB = false;
    public static boolean changed1A = false;
    public static boolean changed2A = false;
    public static boolean changed2B = false;
    public static boolean changed2Y = false;
    public static boolean isSorting = false;
    public static boolean isShooting = false;
    public static double onRampPassive = 0.81;
    public static double onRampPush = 1;
    public static double offRampPush = 0.5;
    public static double offRampPassive = 0;

    /* old pid constants
    public static double shootP = 1.2, shootI = 2.0, shootD = 0.001, shootF = 0;
     */
    ElapsedTime sortTime = new ElapsedTime();
    public static double sort1 = 0.3;
    public static double sort2 = 1;
    public static double sort3 = 1;
    public static double sort4 = 1;
    public static double sort5 = 1;

    public SortSteps sortSteps;
    public enum SortSteps{
        READY, PUSHOFF, PUSHBACK, UP, PUSHON, BACKOFF
    }


    /*
    Object array which holds ball positions [0] to [2] and sort-cycles [3] need to move requiredArtifact to pos [1]
     */
    SortLogic sortLogic = new SortLogic();
    Object[] sortData = new String[4];

    @Override
    public void runOpMode() throws InterruptedException{
        robot = new RobotQuals(hardwareMap);
        //robot.belt.setTargetPosition(robot.belt.getCurrentPosition());
        //robot.belt.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

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
                    -gamepad1.left_stick_y*0.75,
                    -gamepad1.left_stick_x*0.75,
                    -gamepad1.right_stick_x*0.4,
                    true
            );

            //intake and reverse intake
            if(!isSorting){
                if(gamepad2.right_bumper){
                    robot.belt.setPower(beltOn);
                    robot.intake.setPower(intakeOn);
                }
                else if(gamepad2.x){
                    robot.belt.setPower(beltReverse);
                    robot.flyRight.setDirection(DcMotorEx.Direction.FORWARD);
                    robot.flyLeft.setDirection(DcMotorEx.Direction.REVERSE);
                    robot.shooterPIDF(velocity);
                    robot.intake.setPower(-0.7);
                }
                else if(gamepad2.left_bumper){
                    robot.belt.setPower(beltReverse);
                    robot.flyRight.setDirection(DcMotorEx.Direction.FORWARD);
                    robot.flyLeft.setDirection(DcMotorEx.Direction.REVERSE);
                    robot.shooterPIDF(velocity);
                    robot.intake.setPower(0);
                }
                else{
                    robot.flyRight.setDirection(DcMotorEx.Direction.REVERSE);
                    robot.flyLeft.setDirection(DcMotorEx.Direction.FORWARD);
                    robot.belt.setPower(0);
                    robot.intake.setPower(0);
                    robot.flyRight.setPower(0);
                    robot.flyLeft.setPower(0);
                }
            }




            //physical sort method
            //TODO: FIGURE OUT WHAT TO DO WITH THE FIRST BALL...THIS CYCLES ASSUMING ONE IS ALREADY IN THE BACK RAMP
            //TODO: UNCOMMENT ONCE SERVO POSITIONS ARE GOOD
            /// THE "//" commented out code has to do with the belt motor...
            /// ...running to position, aka need to test
            /*
            switch(sortSteps){
                case READY:
                    if(gamepad2.y && !changed2Y){
                        changed2Y= true;
                        isSorting = true;
                        robot.belt.setPower(0);
                        //robot.belt.setTargetPosition(robot.belt.getCurrentPosition());
                        //robot.belt.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        robot.intake.setPower(0);
                        robot.offRamp.setPosition(offRampPassive);
                        robot.onRamp.setPosition(onRampPassive);
                        sortTime.reset();
                        sortSteps = SortSteps.PUSHOFF;
                    }
                    else if(!gamepad2.y){
                        changed2Y = false;
                    }
                    break;
                case PUSHOFF:
                    if(!gamepad2.y){
                        changed2Y = false;
                    }
                    if(sortTime.seconds() >= sort1){
                        robot.onRamp.setPosition(onRampPush);
                        sortTime.reset();
                        sortSteps = SortSteps.PUSHBACK;
                    }
                    break;
                case PUSHBACK:
                    if(sortTime.seconds() >= sort2){
                        robot.onRamp.setPosition(onRampPassive);
                        sortTime.reset();
                        sortSteps = SortSteps.UP;
                    }
                    break;
                case UP:
                    if(sortTime.seconds() >= sort3){
                        robot.belt.setPower(1);
                        //robot.belt.setTargetPosition(robot.belt.getCurrentPosition() + beltIncrement);
                        sortTime.reset();
                        sortSteps = SortSteps.PUSHON;
                    }
                    break;
                case PUSHON:
                    if(sortTime.seconds() >= sort4){
                        robot.belt.setPower(0);
                        robot.offRamp.setPosition(offRampPush);
                        sortTime.reset();
                        sortSteps = SortSteps.BACKOFF;
                    }
                    break;
                case BACKOFF:
                    if(sortTime.seconds() >= sort4){
                        robot.offRamp.setPosition(offRampPassive);
                        //robot.belt.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                        isSorting = false;
                        sortTime.reset();
                        sortSteps = SortSteps.READY;
                    }
                    break;
            }

             */

            //New shooting by emad:
            //Still incomplete needs a lot more work - emad
            //Shoot green
            if(gamepad2.b){
                isShooting = true;
                robot.shooterPIDF(velocity);
                //need to have an if statement:
                sortData = sortLogic.updateShotArtifact(sortData);
            } else if(!gamepad2.b){
                //changed2B = false;
                isShooting = false;
                robot.flyRight.setPower(0);
                robot.flyLeft.setPower(0);
            }

            if(gamepad2.y){
                robot.onRamp.setPosition(onRampPush);
            }
            else if(!gamepad2.y){
                robot.onRamp.setPosition(onRampPassive);
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

            telemetryM.debug("Auto Velocity", velocity);
            //telemetryM.addData("velocity left", robot.flyLeft.getVelocity());
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
