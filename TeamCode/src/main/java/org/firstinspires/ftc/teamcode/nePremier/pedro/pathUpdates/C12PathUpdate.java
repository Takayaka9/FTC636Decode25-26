package org.firstinspires.ftc.teamcode.nePremier.pedro.pathUpdates;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.nePremier.pedro.paths.C12Paths;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.Alliance;
import org.firstinspires.ftc.teamcode.nePremier.utils.pedroUtils.BasePathUpdate;

public class C12PathUpdate extends BasePathUpdate {
    final C12Paths p;
    public C12PathUpdate(Alliance alliance, HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap,telemetry, alliance);
        p = new C12Paths(follower, alliance);
    }

    @Override
    public void autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                follower.followPath(p.startToShoot);
                pathState = 1;
                break;
            case 1:
                if (!follower.atParametricEnd()) {
                    shootTimer.resetThenStart();
                    shootControl.run();
                    pathState = 2;
                }
                break;
            case 2:
                if (shootTimer.checkFinished()) {
                    shootControl.stop();
                    follower.followPath(p.preIntakeSpike2);
                    intakeControl.run();
                    pathState = 3;
                }
                break;
            case 3:
                if (follower.atParametricEnd()) {
                    follower.followPath(p.intakeSpike2);
                    pathState = 4;
                }
                break;
            case 4:
                if (follower.atParametricEnd()) {
                    follower.followPath(p.spike2ToShoot);
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
                    follower.followPath(p.preIntakeSpike1);
                    intakeControl.run();
                    pathState = 7;
                }
                break;
            case 7:
                if (follower.atParametricEnd()) {
                    follower.followPath(p.intakeSpike1);
                    pathState = 8;
                }
                break;
            case 8:
                if (follower.atParametricEnd()) {
                    follower.followPath(p.spike1toShoot);
                    pathState = 9;
                }
                break;
            case 9:
                if (follower.atParametricEnd()) {
                    intakeControl.stop();
                    shootTimer.resetThenStart();
                    shootControl.run();
                    pathState = 10;
                }
                break;
            case 10:
                if (shootTimer.checkFinished()) {
                    shootControl.stop();
                    follower.followPath(p.preIntakeSpike3);
                    intakeControl.run();
                    pathState = 11;
                }
                break;
            case 11:
                if (follower.atParametricEnd()) {
                    follower.followPath(p.intakeSpike3);
                    pathState = 12;
                }
                break;
            case 12:
                if (follower.atParametricEnd()) {
                    follower.followPath(p.spike3toShoot);
                    pathState = 13;
                }
                break;
            case 13:
                if (follower.atParametricEnd()) {
                    intakeControl.stop();
                    shootTimer.resetThenStart();
                    shootControl.run();
                    pathState = 14;
                }
                break;
            case 14:
                if (shootTimer.checkFinished()) {
                    shootControl.stop();
                    pathState = 15;
                }
                break;
            case 15:
                break;
        }
    }
}
