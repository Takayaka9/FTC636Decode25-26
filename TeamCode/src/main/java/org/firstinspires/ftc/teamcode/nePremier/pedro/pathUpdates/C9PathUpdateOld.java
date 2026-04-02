package org.firstinspires.ftc.teamcode.nePremier.pedro.pathUpdates;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.nePremier.pedro.autoConstants.AutoConstants;
import org.firstinspires.ftc.teamcode.nePremier.pedro.paths.CPaths;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.Alliance;
import org.firstinspires.ftc.teamcode.nePremier.utils.pedroUtils.BasePathUpdate;

@Deprecated
public class C9PathUpdateOld extends BasePathUpdate {
    final CPaths paths;
    public C9PathUpdateOld(Alliance alliance, HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap,telemetry, alliance);
        paths = new CPaths(follower, alliance);
    }

    @Override
    public void autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                follower.followPath(paths.startToShoot);
                pathState = 1;
                break;
            case 1:
                if (atPose(paths.nearShootPose)) {
                    shootTimer.reset();
                    shootControl.run();
                    pathState = 2;
                }
                break;
            case 2:
                if (shootTimer.milliseconds() >= AutoConstants.shootTime) {
                    shootControl.stop();
                    follower.followPath(paths.preIntakeSpike2);
                    pathState = 3;
                }
                break;
            case 3:
                if (atPose(paths.intakeP2Pose)) {
                    intakeControl.run();
                    follower.followPath(paths.intakeSpike2);
                    pathState = 4;
                }
                break;
            case 4:
                if (atPose(paths.intake2Pose)) {
                    intakeControl.stop();
                    follower.followPath(paths.spike2ToShoot);
                    pathState = 5;
                }
                break;
            case 5:
                if (atPose(paths.nearShootPose)) {
                    shootTimer.reset();
                    shootControl.run();
                    pathState = 6;
                }
                break;
            case 6:
                if (shootTimer.milliseconds() >= AutoConstants.shootTime) {
                    shootControl.stop();
                    follower.followPath(paths.preIntakeSpike1);
                    pathState = 7;
                }
                break;
            case 7:
                if (atPose(paths.intakeP2Pose)) {
                    intakeControl.run();
                    follower.followPath(paths.intakeSpike1);
                    pathState = 8;
                }
                break;
            case 8:
                if (atPose(paths.intake1Pose)) {
                    intakeControl.stop();
                    follower.followPath(paths.spike1toShoot);
                    pathState = 9;
                }
                break;
            case 9:
                if (atPose(paths.nearShootPose)) {
                    shootTimer.reset();
                    shootControl.run();
                    pathState = 10;
                }
                break;
            case 10:
                if (shootTimer.milliseconds() >= AutoConstants.shootTime) {
                    shootControl.stop();
                    follower.followPath(paths.preIntakeSpike3);
                    pathState = 11;
                }
                break;
            case 11:
                if (atPose(paths.intakeP3Pose)) {
                    intakeControl.run();
                    follower.followPath(paths.intakeSpike3);
                    pathState = 12;
                }
                break;
            case 12:
                if (atPose(paths.intake3Pose)) {
                    intakeControl.stop();
                    follower.followPath(paths.spike3toShoot);
                    pathState = 13;
                }
                break;
            case 13:
                if (atPose(paths.nearShootPose)) {
                    shootTimer.reset();
                    shootControl.run();
                    pathState = 14;
                }
                break;
        }
    }
}
