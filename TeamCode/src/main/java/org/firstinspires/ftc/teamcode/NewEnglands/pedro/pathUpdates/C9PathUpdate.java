package org.firstinspires.ftc.teamcode.NewEnglands.pedro.pathUpdates;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.NewEnglands.pedro.paths.C9Paths;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.alliance.Alliance;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.inputSystem.ControlType;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.pedroUtils.BasePathChain;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.pedroUtils.BasePathUpdate;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.pedroUtils.PathUpdateHelper;

public class C9PathUpdate extends BasePathUpdate {
    C9Paths paths;
    public C9PathUpdate(Alliance alliance, HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap,telemetry, alliance);
        paths = new C9Paths(follower, alliance);
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
                    shootTimer.resetThenStart();
                    shootControl.run();
                    pathState = 2;
                }
                break;
            case 2:
                if (shootTimer.checkFinished()) {
                    shootControl.stop();
                    follower.followPath(paths.preIntakeSpike2);
                    pathState = 3;
                }
            case 3:
                if (atPose(paths.pIntake2Pose)) {
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
                    shootTimer.resetThenStart();
                    shootControl.run();
                    pathState = 6;
                }
                break;
            case 6:
                if (shootTimer.checkFinished()) {
                    shootControl.stop();
                    follower.followPath(paths.preIntakeSpike1);
                    pathState = 7;
                }
                break;
            case 7:
                if (atPose(paths.pIntake2Pose)) {
                    intakeControl.run();
                    follower.followPath(paths.intakeSpike1);
                    pathState = 8;
                }
                break;
            case 8:

        }
    }
}
