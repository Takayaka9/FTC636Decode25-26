package org.firstinspires.ftc.teamcode.nePremier.utils.pedroUtils;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.nePremier.pedro.autoConstants.AutoConstants;
import org.firstinspires.ftc.teamcode.nePremier.pedro.autoConstants.BluePoseLib;
import org.firstinspires.ftc.teamcode.nePremier.pedro.autoConstants.RedPoseLib;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.CurrentAlliance;

public final class AutoHelper {

//    public static void update(BasePathUpdate pathUpdate) {
//        if (pathUpdate.fleeTimer.checkFinished() && !pathUpdate.fled) {
//            pathUpdate.follower.followPath(createFleePath(pathUpdate));
//            pathUpdate.fled = true;
//        } else if (pathUpdate.fleeTimer.checkFinished() && pathUpdate.fled) {
//            pathUpdate.zeroTurret();
//        } else {
//            pathUpdate.autonomousPathUpdate();
//        }
//    }

    public static void update(BasePathUpdate pathUpdate) {
        pathUpdate.autonomousPathUpdate();
        if (pathUpdate.fleeTimer.checkFinished() && !pathUpdate.fled) {
            pathUpdate.fled = true;
        }
        if (pathUpdate.fled) {
            pathUpdate.zeroTurret();
        }
    }

//    public static void update(BasePathUpdate pathUpdate) {
//        pathUpdate.autonomousPathUpdate();
//    }

    public static PathChain createFleePath(BasePathUpdate pathUpdate) {
        Pose fleePose = null;
        if (pathUpdate.follower.getPose().getY() > 38.5) {
            switch (CurrentAlliance.alliance) {
                case RED:
                    fleePose = new Pose(116, 60);
                    break;
                case BLUE:
                    fleePose = new Pose(28, 60);
                    break;
            }
        } else {
            switch (CurrentAlliance.alliance) {
                case RED:
                    fleePose = new Pose(96, 24);
                    break;
                case BLUE:
                    fleePose = new Pose(48, 24);
                    break;
            }
        }

        return pathUpdate.follower.pathBuilder()
                .addPath(new BezierLine(pathUpdate.follower.getPose(), fleePose))
                .setConstantHeadingInterpolation(pathUpdate.follower.getPose().getHeading())
                .setBrakingStrength(0)
                .build();
    }

}
