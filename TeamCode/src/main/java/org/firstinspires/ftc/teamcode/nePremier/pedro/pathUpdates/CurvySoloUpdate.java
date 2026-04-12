package org.firstinspires.ftc.teamcode.nePremier.pedro.pathUpdates;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.nePremier.pedro.autoConstants.AutoConstants;
import org.firstinspires.ftc.teamcode.nePremier.pedro.paths.CurvyPaths;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.Alliance;
import org.firstinspires.ftc.teamcode.nePremier.utils.pedroUtils.AutoHelper;
import org.firstinspires.ftc.teamcode.nePremier.utils.pedroUtils.BasePathUpdate;
public class CurvySoloUpdate extends BasePathUpdate {
    final CurvyPaths p;
    public CurvySoloUpdate(Alliance alliance, HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap,telemetry, alliance);
        p = new CurvyPaths(follower, alliance);
        follower.setPose(p.nearStartPose);
    }

    @Override
    public void autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                follower.followPath(p.startToShoot);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    shoot();
                    setPathState(4);
                }
                break;
            case 2:
                if (checkShoot()) {
                    shootControl.stop();
                    follower.followPath(p.spike2andEmpty);
                    setPathState(90);
                }
                break;
            case 90:
                intakeControl.run();
                setPathState(6);
                break;
//            case 3:
//                if(!follower.isBusy()){
//                    follower.followPath(p.returnToShoot);
//                    setPathState(83);
//                }
//                break;
            case 83:
                if (atPose()) {
                    shoot();
                    setPathState(4);
                }
                break;
            case 4:
                if (checkShoot()) {
                    shootControl.stop();
                    follower.followPath(p.intakeSpike1, AutoConstants.intakeSpeed, true);
                    setPathState(49);
                }
                break;
            case 49:
                intakeControl.run();
                setPathState(67);
                break;
            case 67:
                if (!follower.isBusy() && atPose()) {
                    shoot();
                    setPathState(2);
                }
                break;
            case 6:
                if (checkShoot()) {
                    shootControl.stop();
                    follower.followPath(p.intakeSpike3);
                    setPathState(69);
                }
                break;
            case 69:
                intakeControl.run();
                setPathState(7);
                break;
            case 7:
                if (!follower.isBusy() && atPose()) {
                    shoot();
                    setPathState(8);
                }
                break;
            case 8:
                if (checkShoot()) {
                    follower.followPath(AutoHelper.createFleePath(this));
                    shootControl.stop();
                    setPathState(9);
                }
                break;
            case 9:
                break;
        }
    }
}
