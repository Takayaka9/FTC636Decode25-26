package org.firstinspires.ftc.teamcode.afterPremier.opmodes.tele;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.afterPremier.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.afterPremier.robot.Rico;
import org.firstinspires.ftc.teamcode.afterPremier.util.Alliance;
import org.firstinspires.ftc.teamcode.afterPremier.util.RobotConstants;

public class BaseTeleOp extends BaseOpMode {
    Rico r;
    final Alliance a;
    public BaseTeleOp(Alliance a){
        this.a = a;
    }
    @Override
    public void init() {
        r = new Rico(hardwareMap, a);
        super.init();
        r.f.setStartingPose(RobotConstants.getPose());
        r.t.useLastTurretPos();
    }

    @Override
    public void start() {
        super.start();
        r.f.startTeleOpDrive(false);
    }
    private boolean shooting = false;
    @Override
    public void loop() {
        super.loop();
        r.periodic();

        r.f.setTeleOpDrive(
                -gamepad1.left_stick_y*1,
                -gamepad1.left_stick_x*1,
                -gamepad1.right_stick_x*1,
                true
        );

        if(gamepad1.rightTriggerWasPressed()){
            r.shoot().schedule();
            shooting = true;
        }
        else if(!(gamepad1.right_trigger < 0.2)){
            shooting = false;
        }

        if(gamepad2.rightBumperWasPressed()){
            r.t.incrementRight().schedule();
        }
        else if(gamepad2.leftBumperWasPressed()){
            r.t.incrementLeft().schedule();
        }

        //intake behavior
        if(shooting){
            return;
        }
        if(gamepad1.right_bumper){
            r.i.in().schedule();
        }
        else if(gamepad1.left_bumper){
            r.i.out().schedule();
        }
        else{
            r.i.off().schedule();
        }
    }
}
