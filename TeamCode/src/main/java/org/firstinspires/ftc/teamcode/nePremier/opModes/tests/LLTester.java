package org.firstinspires.ftc.teamcode.nePremier.opModes.tests;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
@Disabled

public class LLTester extends OpMode {
    Limelight3A l;
    Follower follower;
    TelemetryManager t;
    @Override
    public void init() {
        l = hardwareMap.get(Limelight3A.class, "limelight");
        l.pipelineSwitch(3);
        follower = Constants.createFollower(hardwareMap);
        t = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void start() {
        l.start();
    }

    @Override
    public void loop() {
        follower.update();
        l.updateRobotOrientation(Math.toDegrees(follower.getPose().getHeading()) + 90);
        LLResult result = l.getLatestResult();
        if(result != null){
            if (result.isValid()) {
                //converts poses
                Pose3D llPose = result.getBotpose_MT2();
                Pose2D conversionPose = new Pose2D(DistanceUnit.INCH, llPose.getPosition().x, llPose.getPosition().y, AngleUnit.RADIANS, llPose.getOrientation().getYaw(AngleUnit.RADIANS));
                Pose ftcStandard = PoseConverter.pose2DToPose(conversionPose, InvertedFTCCoordinates.INSTANCE);
                Pose pedroPose = ftcStandard.getAsCoordinateSystem(PedroCoordinates.INSTANCE);

                //telemetry
                t.addData("limelight pose", llPose.toString());
                t.addData("pp pose CURRENTLY BEING SET TO LL - emad", pedroPose.toString());
                t.addData("follower pose", follower.getPose());

            } else {
                t.addLine("Pose is not null, but invalid");
            }
        } else {
            t.addLine("pose was null");
        }
    }
}
