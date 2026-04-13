package org.firstinspires.ftc.teamcode.nePremier.pedro.pathUpdates;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.nePremier.pedro.paths.FarPaths;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.Alliance;
import org.firstinspires.ftc.teamcode.nePremier.utils.pedroUtils.AutoHelper;
import org.firstinspires.ftc.teamcode.nePremier.utils.pedroUtils.BasePathUpdate;

public class FarUpdate extends BasePathUpdate {
    final FarPaths p;
    public FarUpdate(Alliance alliance, HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap,telemetry, alliance);
        p = new FarPaths(follower, alliance);
        follower.setPose(p.farStartPose);
    }

    @Override
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(p.startToShoot);
                setPathState(1);
                break;
            case 1:
                if (atFarPose()) {
                    shoot();
                    setPathState(2);
                }
                break;
            case 2:
                if (checkFarShoot()) {
                    shootControl.stop();
                    follower.followPath(p.intakeSpike3);
                    setPathState(20);
                }
                break;
            case 20:
                intakeControl.run();
                setPathState(3);
                break;
            case 3:
                intakeControl.run();
                if (!follower.isBusy()) {
                    follower.followPath(p.spike3toShoot);
                    setPathState(4);
                }
                break;
            case 4:
                if (atFarPose()) {
                    shoot();
                    setPathState(5);
                }
                break;
            case 5:
                if (checkFarShoot()) {
                    shootControl.stop();
                    follower.followPath(p.shootToWall);
                    setPathState(50);
                }
                break;
            case 50:
                intakeControl.run();
                setPathState(6);
                break;
            case 6:
                intakeControl.run();
                if(!follower.isBusy()){
                    follower.followPath(p.wallToShoot);
                    setPathState(7);
                }
                break;
            case 7:
                if(atFarPose()){
                    shoot();
                    setPathState(8);
                }
                break;
            case 8:
                if (checkFarShoot()) {
                    shootControl.stop();
                    follower.followPath(p.shootToFarIntake);
                    setPathState(80);
                }
                break;
            case 80:
                intakeControl.run();
                setPathState(9);
                break;
            case 9:
                intakeControl.run();
                if(!follower.isBusy()) { //TODO: change to timer as needed
                    follower.followPath(p.farIntakeToShoot);
                    setPathState(10);
                }
                break;
            case 10:
                if(atFarPose()){
                    shoot();
                    setPathState(11);
                }
                break;
            case 11:
                if (checkFarShoot()) {
                    shootControl.stop();
                    pathState = 12;
                }
                break;
            case 12:
                follower.followPath(AutoHelper.createFleePath(this));
                break;
        }
    }
}
