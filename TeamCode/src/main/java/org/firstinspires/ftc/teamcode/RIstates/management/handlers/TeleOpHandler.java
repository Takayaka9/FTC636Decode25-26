package org.firstinspires.ftc.teamcode.RIstates.management.handlers;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RIstates.management.Systems.servo.LiftServo;
import org.firstinspires.ftc.teamcode.RIstates.management.fsm.FSM;

public class TeleOpHandler {
    private final FSM fsm;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    private final ShooterHandler shooterHandler;
    private final LimelightHandler limelight;
    private final LiftServo lift;

    private final Follower follower;

    private Pose blueParkPose = null;
    private Pose redParkPose = null;
    
    private int alliance = 0;


    public TeleOpHandler(FSM fsm, Gamepad gamepad1, Gamepad gamepad2, ShooterHandler shooterHandler, LimelightHandler limelight, LiftServo liftServo, Follower follower, int alliance) {
        this.fsm = fsm;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.shooterHandler = shooterHandler;
        this.limelight = limelight;
        this.lift = liftServo;
        this.follower = follower;
        blueParkPose = new Pose(105.35, 37.8, Math.toRadians(90));
        redParkPose = new Pose(38.6, 37.8, Math.toRadians(90));
        this.alliance = alliance;
    }

    private boolean changedA = false;
    public boolean changedB = false;
    public boolean changed1B = false;
    public boolean changedX = false;
    private boolean changedRT = false;
    private boolean changedLT = false;
    private boolean changed1A = false;
    private boolean allianceSelecting = false;
    public boolean updateLimelight = false;
    public boolean updateLift = false;
    private boolean off = false;

    private FSM.StateName requestingTransition = null;
    public void setTransition(FSM.StateName stateName){
            requestingTransition = stateName;
    }



    public void start(){
        fsm.runNew(FSM.StateName.Norm);
    }
    public void stop(){
        fsm.runNew(FSM.StateName.Norm);
    }

    public void update() {
        fsm.update();
        /*
        if statement order determines priority
        logic concept
        if inputs && not already active then request transition
        if no inputs && already active then stop state
         */

        //localization backup
        if (gamepad1.b && !changed1B) {
            if (alliance == 1) {
                follower.setPose(blueParkPose);
            } 
            if (alliance == 2) {
                follower.setPose(redParkPose);
            }
            changed1B = true;
        }
        if (!gamepad1.b && changed1B) {
            changed1B = false;
        }

        //limelight, not through fsm
        if (gamepad1.a && !changed1A) {
            changed1A = true;
            updateLimelight = true;
        }
        if (updateLimelight) {
            limelight.updatePosition();
            if (limelight.checkFound()) {
                updateLimelight = false;
                changed1A = false;
            }
        }

        //lift code
        if (gamepad2.dpad_up) {
            updateLift = true;
        }
        if (updateLift) {
            lift.up();
        }
        if (gamepad2.dpad_down) {
            lift.down();
            updateLift = false;
        }
        if (gamepad2.dpad_left) {
            updateLift = false;
            lift.stop();
        }


        //Alliance select state
        if (gamepad1.left_trigger > 0.3 && gamepad1.right_trigger > 0.3 && !allianceSelecting) {
            setTransition(FSM.StateName.AllianceSelect);
            allianceSelecting = true;
            off = false;
        } else if (gamepad1.left_trigger < 0.3 & gamepad1.right_trigger < 0.3 && allianceSelecting) {
            allianceSelecting = false;
            off = true;
        }

        //different logic which checks shootRunning for stop
//        if (gamepad2.a && !changedA) {
//            changedA = true;
//            setTransition(FSM.StateName.Shoot);
//            off = false;
//        } else if (shooterHandler.shooterRunning && changedA) {
//            changedA = false;
//            off = true;
//        }
        if (gamepad2.a && !changedA) {
            changedA = true;
            setTransition(FSM.StateName.Shoot);
            off = false;
        } else if (!gamepad2.a && changedA) {
            changedA = false;
            off = true;
        }

        //backout
        if (gamepad2.left_bumper && !changedLT) {
            changedLT = true;
            setTransition(FSM.StateName.Backout);
            off = false;
        } else if (!gamepad2.left_bumper && changedLT) {
            changedLT = false;
            off = true;
        }

        //intake
        if (gamepad2.right_bumper && !changedRT) {
            changedRT = true;
            setTransition(FSM.StateName.Intake);
            off = false;
        } else if (!gamepad2.right_bumper && changedRT) {
            changedRT = false;
            off = true;
        }

//        transition to next state
        if (off) {
            setTransition(FSM.StateName.Norm);
        }

        if (requestingTransition != null) {
            fsm.runNew(requestingTransition);
        }
        requestingTransition = null;

    }
}

