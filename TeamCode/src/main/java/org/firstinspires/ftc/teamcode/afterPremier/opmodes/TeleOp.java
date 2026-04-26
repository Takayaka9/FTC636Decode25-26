package org.firstinspires.ftc.teamcode.afterPremier.opmodes;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.afterPremier.robot.Rico;
import org.firstinspires.ftc.teamcode.afterPremier.util.Alliance;
import org.firstinspires.ftc.teamcode.afterPremier.util.BaseOpMode;

public class TeleOp extends BaseOpMode {
    Rico r;
    final Alliance a;
    Follower f;
    public TeleOp(Alliance a){
        this.a = a;
    }
    @Override
    public void init() {
        super.init();
        r = new Rico(hardwareMap, a);
    }

    @Override
    public void loop() {
        super.loop();
        r.t.aim(r.goalPose, f.getPose());
        r.h.angleHood(f.getPose(), r.goalPose);
        if(gamepad1.right_bumper){
            r.i.in();
        }
        else if(gamepad1.left_bumper){
            r.i.out();
        }
        else{
            r.i.off();
        }
    }
}
