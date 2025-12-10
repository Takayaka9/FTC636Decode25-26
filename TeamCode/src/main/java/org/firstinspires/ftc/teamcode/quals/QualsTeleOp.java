package org.firstinspires.ftc.teamcode.quals;

import org.firstinspires.ftc.teamcode.PIDFControl_ForVelocity;
import org.firstinspires.ftc.teamcode.Sorting.SortLogic;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    public static double velocity = 40; //TODO: test ts
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
    public static boolean isMacroing = false;
    public static boolean isShooting = false;
    public static boolean shootToggle = false;
    public static double onRampPassive = 0.44;
    public static double onRampPush = 0.8;
    public static double offRampPush = 0.8;
    public static double offRampPassive = 0.45;
    public double integralSum;
    public double lastError;
    ElapsedTime pidTime = new ElapsedTime();

    /* old pid constants
    public static double shootP = 1.2, shootI = 2.0, shootD = 0.001, shootF = 0;
     */
    //timer values etc. for sorting macro
    ElapsedTime sortTime = new ElapsedTime();
    public static double sort1 = 0;
    public static double sort2 = 0.6;
    public static double sort3 = 0.1;
    public static double sort4 = 0.3;
    public static double sort5 = 0.5;
    public enum SortSteps{
        READY, PUSHOFF, PUSHBACK, UP, PUSHON, BACKOFF
    }
    public SortSteps sortSteps = SortSteps.READY;
    //timer values etc. for shooting macro
    ElapsedTime shootTime = new ElapsedTime();
    public static double shoot1 = 1;
    public static double shoot2 = 0.3;
    public static double shoot3 = 1;
    public static double shoot4 = 0.4;
    public static double shoot5 = 1;
    public static double shoot6 = 1;
    public enum ShootSteps{
        READY, REV_1, SHOOT_1, REV_2, SHOOT_2, REV_3, SHOOT_3, FINISH
    }
    public ShootSteps shootSteps = ShootSteps.READY;
    /*
    Object array which holds ball positions [0] to [2] and sort-cycles [3] need to move requiredArtifact to pos [1]
     */
    SortLogic sortLogic = new SortLogic();
    Object[] sortData = new String[4];
    public static double Kp = 0.1;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double Kf = 0;
    //PIDFControl_ForVelocity control = new PIDFControl_ForVelocity(kP, kI, kD, kF);

    @Override
    public void runOpMode() throws InterruptedException{
        robot = new RobotQuals(hardwareMap);
        //robot.belt.setTargetPosition(robot.belt.getCurrentPosition());
        //robot.belt.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        waitForStart();
        shootToggle = false;

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

            if (gamepad1.left_trigger < 0.3) {
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y*1,
                        -gamepad1.left_stick_x*1,
                        -gamepad1.right_stick_x*0.55,
                        true
                );
            }
            if (gamepad1.left_trigger > 0.3) {
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y*0.35,
                        -gamepad1.left_stick_x*0.35,
                        -gamepad1.right_stick_x*0.2,
                        true
                );
            }

            //intake and reverse intake
            if(!isMacroing){
                if(gamepad2.right_bumper){
                    robot.belt.setPower(beltOn);
                    robot.intake.setPower(intakeOn);
                }
                else if(gamepad2.left_trigger > 0.5){
                    robot.belt.setPower(-1);
                    robot.flyLeft.setPower(-1);
                    robot.flyRight.setPower(-1);
                    robot.intake.setPower(-0.7);
                }
                else if(gamepad2.left_bumper){
                    robot.belt.setPower(beltReverse);
                    robot.flyLeft.setPower(-1);
                    robot.flyRight.setPower(-1);
                    robot.intake.setPower(0);
                }
                else if(!gamepad2.left_bumper && gamepad2.left_trigger < 0.5 && !gamepad2.right_bumper){
                    if(!shootToggle){
                        robot.flyRight.setPower(0);
                        robot.flyLeft.setPower(0);
                    }
                    robot.belt.setPower(0);
                    robot.intake.setPower(0);
                    //robot.flyRight.setPower(0);
                    //robot.flyLeft.setPower(0);
                }
            }

            //physical sort method
                //SortData[] holds other ball positions which could be used in an if,
                //I think we should move first ball to pos 1
            /// THE "//" commented out code has to do with the belt motor...
            /// ...running to position, aka need to test

            switch(sortSteps){
                case READY:
                    if(gamepad2.y && !changed2Y){
                        changed2Y= true;
                        isMacroing = true;
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
                    robot.onRamp.setPosition(onRampPush);
                    if(!gamepad2.y){
                        changed2Y = false;
                    }
                    if(sortTime.seconds() >= sort1){
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
                    if(sortTime.seconds() >= sort5){
                        robot.offRamp.setPosition(offRampPassive);
                        //robot.belt.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                        isMacroing = false;
                        sortTime.reset();
                        sortSteps = SortSteps.READY;
                    }
                    break;
            }

            //New shooting by emad:
            //Still incomplete needs a lot more work - emad
            //Shoot green
            /*
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
             */
            //changed to toggle
            /*
            if(gamepad2.b){
                if(!changed2B){
                    integralSum = 0;
                    lastError = 0;
                    isShooting = true;
                    changed2B = true;
                    pidTime.reset();
                }
                double error = velocity-robot.flyRight.getVelocity();
                integralSum += error* pidTime.seconds();
                double derivative = (error- lastError)/ pidTime.seconds();
                lastError = error;

                pidTime.reset();

                double output; // basically the same as the normal PIDControl
                output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (velocity * Kf);

                robot.flyRight.setPower(Math.max(-1, Math.min(1, output))); //clamping so values do not exceed 1 or -1
                robot.flyLeft.setPower(Math.max(-1, Math.min(1, output)));
            }
            else if(!gamepad2.b){
                //changed2B = false;
                isShooting = false;
                changed2B = false;
                robot.flyRight.setPower(0);
                robot.flyLeft.setPower(0);
            }
             */

            if(gamepad2.b && !shootToggle && !changed2B){
                integralSum = 0;
                lastError = 0;
                isShooting = true;
                changed2B = true;
                pidTime.reset();
                shootToggle = true;
                changed2B = true;
            }
            else if(gamepad2.b && shootToggle && !changed2B){
                shootToggle = false;
                changed2B = true;
            }
            else if(!gamepad2.b){
                changed2B = false;
            }

            if(shootToggle){
                double error = velocity-(robot.flyRight.getVelocity());
                integralSum += error* pidTime.seconds();
                double derivative = (error- lastError)/ pidTime.seconds();
                lastError = error;

                pidTime.reset();

                double output; // basically the same as the normal PIDControl
                output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (velocity * Kf);
                telemetryM.addData("output", output);

                robot.flyRight.setPower(Math.max(-1, Math.min(1, output))); //clamping so values do not exceed 1 or -1
                robot.flyLeft.setPower(Math.max(-1, Math.min(1, output)));
                //robot.flyRight.setPower(1);
                //robot.flyLeft.setPower(1);
                telemetryM.addData("motor power", Math.max(-1, Math.min(1, output)));
            }
            else{
                robot.flyRight.setPower(0);
                robot.flyLeft.setPower(0);
            }

            //macro to shoot three with one button press...hopefully
            //TODO: make dependant on upper color sensor distance data
            //I dont think it will work otherwise because it just wont be consistent
            switch(shootSteps){
                case READY:
                    if(gamepad2.a && !changed2A){
                        changed2A = true;
                        isMacroing = true;
                        //isShooting = true;
                        //robot.flyRight.setPower(0);
                        //robot.flyLeft.setPower(0);
                        robot.belt.setPower(0);
                        //robot.belt.setTargetPosition(robot.belt.getCurrentPosition());
                        //robot.belt.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        robot.intake.setPower(0);
                        robot.offRamp.setPosition(offRampPassive);
                        robot.onRamp.setPosition(onRampPassive);
                        shootTime.reset();
                        shootSteps = ShootSteps.SHOOT_1;
                    }
                    else if(!gamepad2.a){
                        changed2A = false;
                    }
                    break;
                case SHOOT_1:
                    robot.belt.setPower(1);
                    //robot.shooterPIDF(velocity);
                    //robot.belt.setTargetPosition(robot.belt.getCurrentPosition() + beltIncrement);
                    shootTime.reset();
                    shootSteps = ShootSteps.REV_2;
                    if(!gamepad2.a){
                        changed2A = false;
                    }
                    break;
                case REV_2:
                    if(shootTime.seconds() >= shoot2){
                        robot.belt.setPower(0);
                        robot.offRamp.setPosition(offRampPush);
                        //robot.shooterPIDF(velocity);
                        shootTime.reset();
                        shootSteps = ShootSteps.SHOOT_2;
                    }
                    if(!gamepad2.a){
                        changed2A = false;
                    }
                    break;
                case SHOOT_2:
                    if(shootTime.seconds() >= shoot3){
                        robot.offRamp.setPosition(offRampPassive);
                        //robot.shooterPIDF(velocity);
                        robot.belt.setPower(1);
                        shootTime.reset();
                        shootSteps = ShootSteps.REV_3;
                    }
                    break;
                case REV_3:
                    if (shootTime.seconds() >= shoot4) {
                        robot.belt.setPower(0);
                        //robot.shooterPIDF(velocity);
                        shootTime.reset();
                        shootSteps = ShootSteps.SHOOT_3;
                    }
                    break;
                case SHOOT_3:
                    if(shootTime.seconds() >= shoot5){
                        robot.belt.setPower(1);
                        //robot.shooterPIDF(velocity);
                        shootTime.reset();
                        shootSteps = ShootSteps.FINISH;
                    }
                    break;
                case FINISH:
                    if(shootTime.seconds()>= shoot6){
                        robot.belt.setPower(0);
                        //robot.flyLeft.setPower(0);
                        //robot.flyRight.setPower(0);
                        isMacroing = false;
                        //isShooting = false;
                        shootSteps = ShootSteps.READY;
                    }
                    break;
            }

            if(!isMacroing){
                if(gamepad2.x){
                    robot.onRamp.setPosition(onRampPush);
                }
                else if(!gamepad2.x){
                    robot.onRamp.setPosition(onRampPassive);
                }
                if(gamepad2.dpad_right){
                    robot.offRamp.setPosition(offRampPush);
                }
                else if(!gamepad2.dpad_right){
                    robot.offRamp.setPosition(offRampPassive);
                }
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

            telemetryM.debug("target", velocity);
            telemetryM.addData("target", velocity);
            //telemetryM.addData("velocity left", robot.flyLeft.getVelocity());
            telemetryM.addData("velocity right", robot.flyRight.getVelocity());
            telemetryM.debug("belt power", beltOn);
            telemetryM.debug("intake power", intakeOn);
            telemetryM.addData("intake toggle", intakeToggle);
            telemetryM.addData("belt target", beltTargetPosition);
            telemetryM.debug("belt increment", beltIncrement);
            telemetryM.addData("sortData", sortData);
            telemetryM.addData("shoot toggle", shootToggle);
            //telemetryM.addData("PIDF+FF Output", output);
            telemetryM.addData("Sort Step", sortSteps);
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