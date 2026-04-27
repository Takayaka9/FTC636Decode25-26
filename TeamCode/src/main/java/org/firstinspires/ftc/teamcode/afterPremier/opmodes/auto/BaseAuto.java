package org.firstinspires.ftc.teamcode.afterPremier.opmodes.auto;

import org.firstinspires.ftc.teamcode.afterPremier.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.afterPremier.robot.Rico;
import org.firstinspires.ftc.teamcode.afterPremier.util.Alliance;
import org.firstinspires.ftc.teamcode.afterPremier.util.RobotConstants;

public class BaseAuto extends BaseOpMode {
    public BaseAuto(Alliance a){
        this.a = a;
    }
    Rico r;
    Alliance a;
    @Override
    public void init() {
        super.init();
        r = new Rico(hardwareMap, a);
    }

    @Override
    public void loop() {
        super.loop();
        r.periodic();

        //stores turret and robot positions
        RobotConstants.turretPosTransfer = r.t.getPosition();
        RobotConstants.setPose(r.f.getPose());
    }
}
