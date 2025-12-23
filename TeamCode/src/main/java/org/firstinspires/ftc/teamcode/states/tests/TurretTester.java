package org.firstinspires.ftc.teamcode.states.tests;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.states.RobotStates;
import org.firstinspires.ftc.teamcode.states.subsystems.Turret;

public class TurretTester extends OpMode {
    //RobotStates robot;
    Follower follower;
    TelemetryManager telemetryManager;
    Turret turret;
    Pose startPose = new Pose(0, 0, Math.toRadians(90));
    boolean changedX = false;
    boolean isRed = true;
    @Override
    public void init() {
        //robot = new RobotStates(hardwareMap);
        turret = new Turret(hardwareMap, "turret");
        follower = Constants.createFollower(hardwareMap);
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
        follower.startTeleopDrive();
        follower.setPose(startPose);
    }

    @Override
    public void loop() {
        follower.update();
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y*1,
                -gamepad1.left_stick_x*1,
                -gamepad1.right_stick_x*0.45, false
        );

        //code to constantly follow one of the goals
        if(gamepad1.x && !changedX && isRed){
            turret.trackGoal(1, follower);
            changedX = true;
            isRed = false;
        }
        else if(gamepad1.x && !changedX && !isRed){
            turret.trackGoal(2, follower);
            changedX = true;
            isRed = true;
        }
        else if(!gamepad1.x){
            changedX = false;
        }

        //code to (in theory) snap to blue goal when x is pressed and go back to zero if not
        /*
        if(gamepad1.x){
            turret.trackGoal(1, follower);
        }
        else{
            turret.turnTurret(0);
            //robot.motor.setPower(0); // or set turret motor power to zero
        }

         */
    }

    @Override
    public void stop() {
        super.stop();
    }
}
