package org.firstinspires.ftc.teamcode.NewEnglands.robot.commands;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.NewEnglands.pedroPathing.Drawing;
import org.firstinspires.ftc.teamcode.NewEnglands.robot.systems.Limelight;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.BaseCommand;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.CommandLoop;

public class LLReloc extends BaseCommand {
    private int state;
    public static int motifID;
    public static boolean found;
    TelemetryManager telemetryM;
    Follower follower;
    Limelight limelight;
    Gamepad gamepad1;
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
        follower.startTeleopDrive();
        found = false;
    }

    //states: motif (1), relocalization(3)
    @Override
    public void loop() {
        follower.update();
        follower.setTeleOpDrive(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);

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
                Pose2D aprilTag = new Pose2D(DistanceUnit.INCH, llPose.getPosition().x, llPose.getPosition().y, AngleUnit.RADIANS, llPose.getOrientation().getYaw(AngleUnit.RADIANS));
                Pose ftcStandard = PoseConverter.pose2DToPose(aprilTag, InvertedFTCCoordinates.INSTANCE);
                Pose pedroPose = ftcStandard.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
                telemetryM.addData("pp pose", pedroPose.toString());
                telemetryM.addData("follower pose", follower.getPose());
                //follower.setPose(pedroPose);
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

