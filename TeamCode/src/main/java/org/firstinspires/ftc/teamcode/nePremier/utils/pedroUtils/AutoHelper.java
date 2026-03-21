package org.firstinspires.ftc.teamcode.nePremier.utils.pedroUtils;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.nePremier.pedro.autoConstants.AutoConstants;
import org.firstinspires.ftc.teamcode.nePremier.pedro.autoConstants.BluePoseLib;
import org.firstinspires.ftc.teamcode.nePremier.pedro.autoConstants.RedPoseLib;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.CurrentAlliance;

public final class AutoHelper {

    public static void update(BasePathUpdate pathUpdate) {
        pathUpdate.updateDependencies();
        if (pathUpdate.fleeTimer.checkFinished() && !pathUpdate.fled) {
            pathUpdate.follower.followPath(createFleePath(pathUpdate));
        } else if (pathUpdate.fleeTimer.checkFinished() && pathUpdate.fled) {
            pathUpdate.zeroTurret();
        } else {
            pathUpdate.autonomousPathUpdate();
        }
    }

    private static PathChain createFleePath(BasePathUpdate pathUpdate) {
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