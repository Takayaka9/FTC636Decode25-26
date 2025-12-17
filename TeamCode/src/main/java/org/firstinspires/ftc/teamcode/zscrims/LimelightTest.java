package org.firstinspires.ftc.teamcode.zscrims;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
You need to mess with Limelight's filters first -- refer to documentation. Check the
game manual to see which apriltag family we are using. If the code doesn't work...you can
rewrite it. Good luck!
 */

@Disabled
@Configurable
@TeleOp(name = "limelight", group = "TeleOp")
public class LimelightTest extends LinearOpMode {
    RobotScrims robot;
    Limelight3A limelight3A;
    TelemetryManager telemetryM;
    public static int tagID;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotScrims(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(0);

        waitForStart();

        limelight3A.start();

        while(opModeIsActive()){
            telemetryM.update();
            LLResult llResult = limelight3A.getLatestResult();
            if(llResult != null && llResult.isValid()){
                java.util.List<LLResultTypes.FiducialResult> fidList = llResult.getFiducialResults();
                if(!fidList.isEmpty()){
                    for (LLResultTypes.FiducialResult fid : fidList) {
                        tagID = fid.getFiducialId();
                        telemetryM.addData("Tag ID", tagID);
                    }
                }
                else{
                    telemetryM.addData("Tag ID", "None Detected");
                }

            }
            else{
                telemetryM.addData("Tag ID", "Result not valid");
            }
        }
    }
}
