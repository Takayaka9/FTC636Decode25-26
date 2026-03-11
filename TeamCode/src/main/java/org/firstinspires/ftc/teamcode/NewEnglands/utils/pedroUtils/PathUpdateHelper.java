package org.firstinspires.ftc.teamcode.NewEnglands.utils.pedroUtils;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.NewEnglands.pedro.AutoConstants;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.alliance.Alliance;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.alliance.CurrentAlliance;

public final class PathUpdateHelper {
    public static void update(BasePathUpdate pathUpdate) {
        pathUpdate.updateDependencies();
        if (pathUpdate.fleeTimer.checkFinished() && !pathUpdate.fled) {
            pathUpdate.follower.followPath(createFleePath(pathUpdate));
        } else if (pathUpdate.fleeTimer.checkFinished() && pathUpdate.fled) {
            //TODO: Zero Turret Command and Control!!!!
        } else {
            pathUpdate.autonomousPathUpdate();
        }
    }
    private static PathChain createFleePath(BasePathUpdate pathUpdate) {
        Pose fleePose = null;
        switch (CurrentAlliance.alliance) {
            case RED:
                fleePose = new Pose(96, 58);
            case BLUE:
                fleePose = new Pose(48, 58);
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