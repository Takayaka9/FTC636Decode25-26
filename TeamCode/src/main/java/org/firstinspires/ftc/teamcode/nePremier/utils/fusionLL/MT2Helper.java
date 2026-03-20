package org.firstinspires.ftc.teamcode.nePremier.utils.fusionLL;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class MT2Helper {

    ///code to use megaTag2 to relocalize robot using follower's heading
    public static Pose loop(Limelight3A ll, Follower follower, TelemetryManager telemetryM) {
        //extra info passed to ll for MT2
        ll.updateRobotOrientation(Math.toDegrees(follower.getPose().getHeading() + Math.PI/2));

        //gets ll result data
        //FILTER IS IN PIPELINE ALREADY
        LLResult result = ll.getLatestResult();

        //string for telemetry
        String poseStatus = "";


        if (result != null) {
            if (result.isValid()) {
                //converts poses
                Pose3D llPose = result.getBotpose_MT2();
                Pose2D conversionPose = new Pose2D(DistanceUnit.INCH, llPose.getPosition().x, llPose.getPosition().y, AngleUnit.RADIANS, llPose.getOrientation().getYaw(AngleUnit.RADIANS));
                Pose ftcStandard = PoseConverter.pose2DToPose(conversionPose, InvertedFTCCoordinates.INSTANCE);
                Pose pedroPose = ftcStandard.getAsCoordinateSystem(PedroCoordinates.INSTANCE);

                //telemetry
                telemetryM.addData("limelight pose", llPose.toString());
                telemetryM.addData("pp pose CURRENTLY BEING SET TO LL - emad", pedroPose.toString());
                telemetryM.addData("follower pose", follower.getPose());


                return pedroPose;
            } else {
                poseStatus = "Pose is not null, but invalid";
            }
        } else {
            poseStatus = "pose was null";
        }

        telemetryM.addData("MT2Helper error", poseStatus);

        return null;
    }
}
