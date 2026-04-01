package org.firstinspires.ftc.teamcode.nePremier.pedro.pathUpdates;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.nePremier.pedro.paths.FPaths;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.Alliance;
import org.firstinspires.ftc.teamcode.nePremier.utils.pedroUtils.BasePathUpdate;

public class F6PathUpdate extends BasePathUpdate {
    final FPaths p;
    public F6PathUpdate(Alliance alliance, HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap,telemetry, alliance);
        p = new FPaths(follower, alliance);
        follower.setStartingPose(p.farStartPose);
        follower.setPose(p.farStartPose);
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
                    follower.followPath(p.intakeSpike3);
                    intakeControl.run();
                    shootControl.stop();
                    pathState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(p.spike3toShoot);
                    pathState = 4;
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    intakeControl.stop();
                    shootTimer.resetThenStart();
                    shootControl.run();
                    pathState = 5;
                }
                break;
            case 5:
                if (shootTimer.checkFinished()) {
                    shootControl.stop();
                    intakeControl.run();
                    follower.followPath(p.shootToWall);
                    pathState = 6;
                }
                break;
            case 6:
                if(!follower.isBusy()){
                    follower.followPath(p.wallToShoot);
                    pathState = 7;
                }
                break;
            case 7:
                if(!follower.isBusy()){
                    intakeControl.stop();
                    shootControl.run();
                    shootTimer.resetThenStart();
                    pathState = 8;
                }
                break;
            case 8:
                if (shootTimer.checkFinished()) {
                    shootControl.stop();
                    intakeControl.run();
                    follower.followPath(p.shootToFarIntake);
                    pathState = 9;
                }
                break;
            case 9:
                if(!follower.isBusy()){ //TODO: change to timer as needed
                    follower.followPath(p.farIntakeToShoot);
                    pathState = 10;
                }
                break;
            case 10:
                if(!follower.isBusy()){
                    intakeControl.stop();
                    shootControl.run();
                    shootTimer.resetThenStart();
                    pathState = 11;
                }
                break;
            case 11:
                break;
        }
    }
}
