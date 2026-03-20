package org.firstinspires.ftc.teamcode.nePremier.robot.commands;

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
//import org.firstinspires.ftc.teamcode.NewEnglands.pedroPathing.Drawing;
import org.firstinspires.ftc.teamcode.nePremier.robot.systems.Limelight;
import org.firstinspires.ftc.teamcode.nePremier.utils.pedroUtils.Drawing;
import org.firstinspires.ftc.teamcode.nePremier.utils.commandUtils.BaseCommand;

@Deprecated
public class LLReloc extends BaseCommand {
    public static boolean found = false;
    private final TelemetryManager telemetryM;
    private final Follower follower;
    private final Limelight limelight;
    private final Gamepad gamepad1;
    public LLReloc(Limelight limelight, Follower follower, Gamepad gamepad1, TelemetryManager telemetryM){
        super();
        this.telemetryM = telemetryM;
        this.follower = follower;
        this.limelight = limelight;
        this.gamepad1 = gamepad1;
        found = false;
        //state = 0;
        //motifID = 0;
    }

    /*
    public void setState(int stateNum){
        state = stateNum;
    }

     */

    //init ll and switches pipeline to 3. sets found to false for llHandler purposes
    @Override
    public void init() {
        limelight.ll.start();
        limelight.ll.pipelineSwitch(3);
        follower.startTeleopDrive();
        found = false;
        Drawing.init();
    }

    //states: motif (1), relocalization(3)
    @Override
    public void loop() {
        follower.update();
        follower.setTeleOpDrive(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
        Drawing.drawDebug(follower);
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
        //code to use megaTag2 to relocalize robot using follower's heading
        limelight.ll.updateRobotOrientation(Math.toDegrees(follower.getPose().getHeading() + Math.PI/2));
        LLResult result = limelight.ll.getLatestResult();
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

    }

    @Override
    public void stop() {
        found = false;
    }
}

