package org.firstinspires.ftc.teamcode.nePremier.pedro.pathUpdates;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.nePremier.pedro.paths.C12Paths;
import org.firstinspires.ftc.teamcode.nePremier.pedro.paths.F6Paths;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.Alliance;
import org.firstinspires.ftc.teamcode.nePremier.utils.pedroUtils.BasePathUpdate;

public class F6PathUpdate extends BasePathUpdate {
    final F6Paths p;
    public F6PathUpdate(Alliance alliance, HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap,telemetry, alliance);
        p = new F6Paths(follower, alliance);
    }

    @Override
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(p.startToShoot);
                pathState = 1;
                break;
            case 1:
                if (!follower.isBusy()) {
                    shootTimer.resetThenStart();
                    shootControl.run();
                    pathState = 2;
                }
                break;
            case 2:
                if (shootTimer.checkFinished()) {
                    shootControl.stop();
                    follower.followPath(p.preIntakeSpike3);
                    intakeControl.run();
                    pathState = 3;
                }
                break;
            case 3:
                if (follower.atParametricEnd()) {
                    follower.followPath(p.intakeSpike3);
                    pathState = 4;
                }
                break;
            case 4:
                if (follower.atParametricEnd()) {
                    follower.followPath(p.spike3toShoot);
                    pathState = 5;
                }
                break;
            case 5:
                if (follower.atParametricEnd()) {
                    intakeControl.stop();
                    shootTimer.resetThenStart();
                    shootControl.run();
                    pathState = 6;
                }
                break;
            case 6:
                if (shootTimer.checkFinished()) {
                    shootControl.stop();
                    pathState = 7;
                }
                break;
            case 7:
                break;

        }
    }
}
