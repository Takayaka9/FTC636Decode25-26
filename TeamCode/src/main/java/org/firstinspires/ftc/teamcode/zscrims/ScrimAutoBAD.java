package org.firstinspires.ftc.teamcode.zscrims;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//什么
//pray

@Disabled
@Configurable
@Autonomous(name = "ScrimAutoBAD", group = "Autonomous")
public class ScrimAutoBAD extends OpMode {
    Follower follower;
    TelemetryManager telemetryM;
    RobotScrims robot;
    private int pathState;
    //poses:
    public double startX = 48;
    public double startY = 135;
    public double scoreX = 65;
    public double scoreY = 90;
    public double scoreA = 136;

    //initialize poses
    private final Pose startPose = new Pose(startX, startY, Math.toRadians(270));
    private final Pose scorePose = new Pose(scoreX, scoreY, Math.toRadians(scoreA));

    private Path scorePreload;
    private PathChain scorePreloadPATH;

    public void buildPaths() {
        scorePreloadPATH = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreloadPATH);
                break;
        }
    }
    //你好！
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }
    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {}

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}
