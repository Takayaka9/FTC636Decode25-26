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
    private boolean automatic = true;
    @Override
    public void loop() {
        super.loop();
        r.telemetry.addData("automatic?", automatic);
        r.telemetry.addData("pose", r.f.getPose());
        if(gamepad1.aWasPressed()){
            automatic = !automatic;
        }
        if(automatic){
            r.periodic();
        }
        else{
            r.f.update();
            r.telemetry.update();
            r.t.turnTurret(0);
            r.fly.run(1100);
            r.h.setPosition(0.85);
        }
        r.f.setTeleOpDrive(
                -gamepad1.left_stick_y*1,
                -gamepad1.left_stick_x*1,
                -gamepad1.right_stick_x*1,
                false, a == Alliance.BLUE ? 180:0
        );

        if(gamepad2.rightTriggerWasPressed()){
            r.shoot().schedule();
            shooting = true;
        }
        else if(!(gamepad2.right_trigger < 0.2)){
            shooting = false;
        }

        if(gamepad1.rightBumperWasPressed()){
            r.t.incrementRight().schedule();
        }
        else if(gamepad1.leftBumperWasPressed()){
            r.t.incrementLeft().schedule();
        }

        //intake behavior
        if(shooting){
            return;
        }
        if(gamepad2.right_bumper){
            r.i.in().schedule();
        }
        else if(gamepad2.left_bumper){
            r.i.out().schedule();
        }
        else{
            r.i.off().schedule();
            r.s.close().schedule();
        }
    }
}
