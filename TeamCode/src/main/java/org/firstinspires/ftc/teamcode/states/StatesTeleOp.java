package org.firstinspires.ftc.teamcode.states;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.states.subsystems.Color;
import org.firstinspires.ftc.teamcode.states.subsystems.Hood;
import org.firstinspires.ftc.teamcode.states.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.states.subsystems.ShooterController;
import org.firstinspires.ftc.teamcode.states.subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Configurable
@TeleOp(name = "States TeleOp", group = "TeleOp")
public class StatesTeleOp extends LinearOpMode {
    Follower follower;
    TelemetryManager telemetryM;
    RobotStates robot;
    Turret turret;
    Hood hood;
    Shooter shooter;
    ShooterController shootControl;
    Color color;
    Limelight3A limelight;
    //Velocities for shooters
    public static double velocity = 1650;
    public static double otherVelocity = 2100;
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
    public static boolean changed2X = false;
    public static boolean shootToggle = false;
    public static double onRampPassive = 0.44;
    public static double onRampPush = 0.8;
    public static double offRampPush = 0.7;
    public static double offRampPassive = 0.45;
    public static boolean changed2RT = false;
    public double integralSum;
    public double lastError;
    ElapsedTime pidTime = new ElapsedTime();

    //timer values etc. for sorting macro
    ElapsedTime sortTime = new ElapsedTime();

    public static double sort1 = 0;
    public static double sort2 = 0.6;
    public static double sort3 = 0.1;
    public static double sort4 = 0.4;
    public static double sort5 = 0.4;
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
    ElapsedTime servoTime = new ElapsedTime();
    public enum TurretModes{
        OFF, BLUE, RED
    }
    TurretModes turretModes = TurretModes.OFF;
    public static double Kp = 0.004;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double Kf = 0.006;

    /*
    public enum allianceColor {
        RED, BLUE, NS
    }

     */
    public static int alliance;
    public int getAlliance(){
        return alliance;
    }
    //public allianceColor alliance = allianceColor.NS;
    boolean selectingColor = false;

    @Override
    public void runOpMode() throws InterruptedException{
        robot = new RobotStates(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        follower.update();
        turret = new Turret(hardwareMap, "turret");
        shooter = new Shooter(hardwareMap, "flyRight", "flyLeft");
        hood = new Hood(hardwareMap, "servo");
        shootControl = new ShooterController(shooter, hood, turret, follower);

        waitForStart();
        shootToggle = false;

        if (isStopRequested()) return;

        //Pedro follower

        follower.update();
        follower.startTeleopDrive();
        //limelight.start();

        while(opModeIsActive()){
            telemetryM.update();
            follower.update();


            //Drive stuff
            if (gamepad1.left_trigger < 0.3) {
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y*1,
                        -gamepad1.left_stick_x*1,
                        -gamepad1.right_stick_x*0.45, false
                );
            }
            if (gamepad1.left_trigger > 0.3) {
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y*0.35,
                        -gamepad1.left_stick_x*0.35,
                        -gamepad1.right_stick_x*0.25,
                        false
                );
            }


            //select alliance color
            /*
            //TODO: lights? to indicate. commented out until turret/hood/shooter is tested
            if (gamepad2.right_bumper && gamepad2.left_bumper) {
                if (!gamepad2.b & !gamepad2.x) {}
                else if (gamepad2.b) {
                    alliance = 1;
                }
                else if (gamepad2.x) {
                    alliance = 2;
                }
            }

             */
            if(gamepad1.x){
                hood.increment(true);
            }
            if(gamepad1.y){
                hood.increment(false);
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



            //turret cases
            //TODO: test turret and uncomment when done!
            /*
            switch(turretModes){
                case OFF:
                    //something.setPower(0);
                    turret.trackGoal(0, follower);
                    if(!gamepad2.x){
                        changed2X = false;
                    }
                    if(alliance == 1 && gamepad2.x && !changed2X){
                        turretModes = TurretModes.BLUE;
                        changed2X = true;
                    }
                    else if(alliance == 2 && gamepad2.x && !changed2X){
                        turretModes = TurretModes.RED;
                        changed2X = true;
                    }
                    break;
                case BLUE:
                    turret.trackGoal(1, follower);
                    if(!gamepad2.x){
                        changed2X = false;
                    }
                    if(gamepad2.x && !changed2X){
                        turretModes = TurretModes.OFF;
                        changed2X = true;
                    }
                    break;
                case RED:
                    turret.trackGoal(2, follower);
                    if(!gamepad2.x){
                        changed2X = false;
                    }
                    if(gamepad2.x && !changed2X){
                        turretModes = TurretModes.OFF;
                        changed2X = true;
                    }
                    break;
            }

            //shooter maybe
            /*
            if(gamepad2.b){
                shootControl.shoot(alliance);
            }
            else if(!gamepad2.b){
                shootControl.off();
            }

             */


            //Shooter
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

            double targetTicks;
            if(gamepad2.right_trigger > 0.3){
                targetTicks = otherVelocity * 28.0 / 60.0;
            }
            else{
                targetTicks = velocity * 28.0 / 60.0;
            }

            //Shooter on/off
            if(shootToggle){
                double error = targetTicks-(robot.flyRight.getVelocity());
                double dt = pidTime.seconds();
                if (dt < 0.0001) dt = 0.0001;
                integralSum += error* dt;
                double derivative = (error- lastError)/ dt;
                lastError = error;

                pidTime.reset();

                double output; // basically the same as the normal PIDControl
                output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (targetTicks * Kf);

                robot.flyRight.setPower(output); //clamping so values do not exceed 1 or -1
                robot.flyLeft.setPower(output);
                //robot.flyRight.setPower(1);
                //robot.flyLeft.setPower(1);
                telemetryM.addData("motor power", Math.max(-1, Math.min(1, output)));
            }


            /*
            //Sorting
            //TODO: sorting needs to work better, probably have to change this system for new bot
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





            //Shoot Macro...my beautiful shoot macro...
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
*/

            /*
            if(!isMacroing){
                if(gamepad2.x){
                    robot.onRamp.setPosition(onRampPush);
                    servoTime.reset();
                }
                else if(!gamepad2.x && servoTime.seconds() >= 0.5){
                    robot.onRamp.setPosition(onRampPassive);
                }
                if(gamepad2.dpad_right){
                    robot.offRamp.setPosition(offRampPush);
                    servoTime.reset();
                }
                else if(!gamepad2.dpad_right && servoTime.seconds() >= 0.5){
                    robot.offRamp.setPosition(offRampPassive);
                }
            }

             */

            //telemetry
            telemetryM.debug("target", velocity);
            telemetryM.addData("target", velocity);
            telemetryM.addData("velocity right", robot.flyRight.getVelocity());
            telemetryM.debug("belt power", beltOn);
            telemetryM.debug("intake power", intakeOn);
            telemetryM.addData("intake toggle", intakeToggle);
            telemetryM.addData("shoot toggle", shootToggle);
            telemetryM.addData("Sort Step", sortSteps);
            //telemetryM.addData("blue distance", shootControl.getTargetDistance(follower, 1));
            //telemetryM.addData("red distance", shootControl.getTargetDistance(follower, 2));
            //telemetryM.addData("turret pos", turret.turretPosition());
        }
    }






}