package org.firstinspires.ftc.teamcode.nePremier.pedro.pathUpdates;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.nePremier.pedro.autoConstants.AutoConstants;
import org.firstinspires.ftc.teamcode.nePremier.pedro.paths.CurvyPaths;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.Alliance;
import org.firstinspires.ftc.teamcode.nePremier.utils.pedroUtils.AutoHelper;
import org.firstinspires.ftc.teamcode.nePremier.utils.pedroUtils.BasePathUpdate;

public class CurvyAllianceUpdate extends BasePathUpdate {
    final CurvyPaths p;
    private int gates = 1;
    public CurvyAllianceUpdate(Alliance alliance, HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap,telemetry, alliance);
        p = new CurvyPaths(follower, alliance);
        follower.setStartingPose(p.nearStartPose);
        gates = 1;
    }
    @Override
    public void autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                follower.followPath(p.startToShoot);
                setPathState(1);
                break;
            case 1:
                if (atPose()) {
                    shoot();
                    setPathState(2);
                }
                break;
            case 2:
                if (checkShoot()) {
                    shootControl.stop();
                    follower.followPath(p.intakeSpike1);
                    setPathState(99);
                }
                break;
            case 99:
                intakeControl.run();
                setPathState(3);
                break;
            case 3:
                if (atPose()) {
                    shoot();
                    setPathState(4);
                }
                break;
            case 4:
                if (checkShoot()) {
                    shootControl.stop();
                    follower.followPath(p.spike2andEmpty);
                    setPathState(49);
                }
                break;
            case 49:
                intakeControl.run();
                setPathState(5);
                break;
            case 5:
                if (atPose()) {
                    shoot();
                    setPathState(60);
                }
                break;
            /// case 60 is intended to receive the robot while it is shooting and then gate cycle
            case 60:
                if (checkShoot()) {
                    shootControl.stop();
                    follower.followPath(p.gateToShoot);
                    setPathState(69);
                }
                break;
            case 69:
                intakeControl.run();
                setPathState(70);
                break;
            case 70:
                if (!follower.isBusy()) {
                    gateTimer.reset();
                    setPathState(80);
                }
                break;
            case 80:
                if (checkGate()) {
                    follower.followPath(p.gateToShoot);
                    setPathState(90);
                }
                break;
            case 90:
                if (atPose()) {
                    shoot();
                    setPathState(100);
                }
                break;
            case 100:
                if (gates >= AutoConstants.gateCycles) {
                    follower.followPath(AutoHelper.createFleePath(this));
                    setPathState(11);
                } else {
                    gates++;
                    setPathState(60);
                }
                break;
            case 11:
                break;
        }
    }
}
