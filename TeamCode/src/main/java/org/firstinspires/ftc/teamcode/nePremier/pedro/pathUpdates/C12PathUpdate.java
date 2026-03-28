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
                if (!follower.isBusy()) {
                    shootTimer.resetThenStart();
                    shootControl.run();
                    pathState = 2;
                }
                break;
            case 2:
                if (shootTimer.checkFinished()) {
                    shootControl.stop();
                    follower.followPath(p.intakeSpike2);
                    intakeControl.run();
                    pathState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(p.spike2andEmpty);
                    pathState = 4;
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    follower.followPath(p.emptyToShoot);
                    pathState = 5;
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    intakeControl.stop();
                    shootTimer.resetThenStart();
                    shootControl.run();
                    pathState = 6;
                }
                break;
            case 67:
                if (shootTimer.checkFinished()) {
                    shootControl.stop();
                    follower.followPath(p.shootToGate);
                    intakeControl.run();
                    pathState = 68;
                }
                break;
            case 68:
                if(!follower.isBusy()){
                    //TODO: add timer
                    pathState = 69;
                }
                break;
            case 69:
                if(!follower.isBusy()){ //TODO: change to timer completion NOT !follower.busy()
                    follower.followPath(p.gateToShoot);
                    pathState = 70;
                }
                break;
            case 70:
                if(!follower.isBusy()){
                    intakeControl.stop();
                    shootTimer.resetThenStart();
                    shootControl.run();
                    pathState = 67; //TODO: change to 6 if implemented
                }
                break;
            case 6:
                if (shootTimer.checkFinished()) {
                    shootControl.stop();
                    follower.followPath(p.intakeSpike1);
                    intakeControl.run();
                    pathState = 7;
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(p.spike1toShoot);
                    pathState = 8;
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    intakeControl.stop();
                    shootTimer.resetThenStart();
                    shootControl.run();
                    pathState = 9;
                }
                break;
            case 9:
                if (shootTimer.checkFinished()) {
                    intakeControl.run();
                    shootControl.stop();
                    follower.followPath(p.intakeSpike3);
                    pathState = 10;
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(p.spike3toShoot);
                    pathState = 11;
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    intakeControl.stop();
                    shootTimer.resetThenStart();
                    shootControl.run();
                    pathState = 12;
                }
                break;
            case 12:
                if (shootTimer.checkFinished()) {
                    shootControl.stop();
                    pathState = 13;
                }
                break;
            case 13:
                break;
        }
    }
}
