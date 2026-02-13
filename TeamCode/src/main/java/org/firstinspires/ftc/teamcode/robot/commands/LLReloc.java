package org.firstinspires.ftc.teamcode.robot.commands;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.robot.systems.Limelight;
import org.firstinspires.ftc.teamcode.utils.commandUtils.BaseCommand;
import org.firstinspires.ftc.teamcode.utils.commandUtils.CommandLoop;

public class LLReloc extends BaseCommand {
    private int state;
    public static int motifID;
    public static boolean found;
    TelemetryManager telemetryM;
    Follower follower;
    Limelight limelight;
    public LLReloc(CommandLoop maps, Limelight limelight, Follower follower, TelemetryManager telemetryM){
        super(maps);
        this.telemetryM = telemetryM;
        this.follower = follower;
        this.limelight = limelight;
        found = false;
        addRequirement(limelight);
        //state = 0;
        //motifID = 0;
    }

    /*
    public void setState(int stateNum){
        state = stateNum;
    }

     */

    //inits ll and switches pipeline to 3. sets found to false for llhandler purposes
    @Override
    public void init() {
        limelight.limelight3A.start();
        limelight.limelight3A.pipelineSwitch(3);
        found = false;
    }

    //states: motif (1), relocalization(3)
    @Override
    public void loop() {
        /*
        if(state == 1){
            limelight.limelight3A.pipelineSwitch(state);
            LLResult result = limelight.limelight3A.getLatestResult();
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


         */
        //code to use megatag2 to relocalize robot using follower's heading
        limelight.limelight3A.updateRobotOrientation(Math.toDegrees(follower.getPose().getHeading() + Math.PI/2));
        LLResult result = limelight.limelight3A.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                Pose3D llPose = result.getBotpose_MT2();
                telemetryM.addData("limelight pose", llPose.toString());
                Pose2D pose2D = new Pose2D(DistanceUnit.INCH, llPose.getPosition().x, llPose.getPosition().y, AngleUnit.DEGREES, llPose.getOrientation().getYaw(AngleUnit.DEGREES));
                Pose pedroPose = PoseConverter.pose2DToPose(pose2D, InvertedFTCCoordinates.INSTANCE);
                follower.setPose(pedroPose);
                found = true;
            }
        }
        /*
        if(state == 3){

        }

         */

        telemetryM.addData("pipeline", state);
    }

    @Override
    public void stop() {
        stop();
        found = false;
    }
}

