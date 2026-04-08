package org.firstinspires.ftc.teamcode.nePremier.utils.pedroUtils;

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
        switch (CurrentAlliance.alliance) {
            case RED:
                fleePose = new Pose(96, 58);
                break;
            case BLUE:
                fleePose = new Pose(48, 58);
                break;
        }

        PathChain fleePath;
        fleePath = pathUpdate.follower.pathBuilder()
                .addPath(new BezierLine(pathUpdate.follower.getPose(), fleePose))
                .setTangentHeadingInterpolation()
                .setBrakingStrength(0)
                .setTValueConstraint(AutoConstants.fleePathTValue)
                .build();
        return fleePath;
    }

}
