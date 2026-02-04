package org.firstinspires.ftc.teamcode.RIstates.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Limelight.LimelightController;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
@Configurable
public class RelocalizationTest extends OpMode {
    Limelight3A limelight;
    Follower follower;
    TelemetryManager t;
    Pose start = new Pose(x, y, h);
    public static double x = 72;
    public static double y = 72;
    public static double h = 0;
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        t = PanelsTelemetry.INSTANCE.getTelemetry();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(3);
    }

    @Override
    public void start() {
        follower.setStartingPose(start);
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        t.update();
        t.addData("follower pose", follower.getPose());

        limelight.updateRobotOrientation(Math.toDegrees(follower.getPose().getHeading() + Math.PI/2));
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                Pose3D llPose = result.getBotpose_MT2();
                t.addData("ll pose", llPose.toString());
                Pose2D pose2D = new Pose2D(DistanceUnit.METER, llPose.getPosition().x, llPose.getPosition().y, AngleUnit.DEGREES, llPose.getOrientation().getYaw(AngleUnit.DEGREES));
                t.addData("ftc pose in x", pose2D.getX(DistanceUnit.INCH));
                t.addData("ftc pose in y", pose2D.getY(DistanceUnit.INCH));
                Pose pedroPose = PoseConverter.pose2DToPose(pose2D, InvertedFTCCoordinates.INSTANCE);
                pedroPose = pedroPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
                t.addData("pp (from ll) pose", pedroPose);
            }
        }
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}
