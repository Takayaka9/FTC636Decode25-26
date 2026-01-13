package org.firstinspires.ftc.teamcode.RIstates.management.Systems.Limelight;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Controller;

import java.util.List;

public class LimelightController extends Limelight implements Controller {
    private int state;
    public static int motifID;
    TelemetryManager telemetryM;
    Follower follower;
    public LimelightController(HardwareMap hardwareMap, String name, Follower f){
        super(hardwareMap, name);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = f;
        state = 0;
        motifID = 0;
    }

    public void setState(int stateNum){
        state = stateNum;
    }

    @Override
    public void init() {
        start();
    }

    //states: motif (1), relocalization(3)
    @Override
    public void update() {
        if(state == 1){
            limelight3A.pipelineSwitch(state);
            LLResult result = limelight3A.getLatestResult();
            if(result != null && result.isValid()){
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for(LLResultTypes.FiducialResult r: fiducialResults){
                    int id = r.getFiducialId();
                    if(id == 21 || id == 22 || id == 23){
                        motifID = id;
                    }
                }
            }
        }
        if(state == 3){
            limelight3A.pipelineSwitch(state);
            limelight3A.updateRobotOrientation(Math.toDegrees(follower.getPose().getHeading() + Math.PI/2));
            LLResult result = limelight3A.getLatestResult();
            if (result != null) {
                if (result.isValid()) {
                    Pose3D llPose = result.getBotpose_MT2();
                    telemetryM.addData("limelight pose", llPose.toString());
                    Pose2D pose2D = new Pose2D(DistanceUnit.INCH, llPose.getPosition().x, llPose.getPosition().y, AngleUnit.DEGREES, llPose.getOrientation().getYaw(AngleUnit.DEGREES));
                    Pose pedroPose = PoseConverter.pose2DToPose(pose2D, InvertedFTCCoordinates.INSTANCE);
                    follower.setPose(pedroPose);
                }
            }
        }

        telemetryM.addData("pipeline", state);
    }

    @Override
    public void end() {
        stop();
    }

    @Override
    public errors updateError() {
        return errors.RUNNING;
    }
}
