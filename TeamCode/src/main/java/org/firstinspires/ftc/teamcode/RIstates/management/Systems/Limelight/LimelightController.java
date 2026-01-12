package org.firstinspires.ftc.teamcode.RIstates.management.Systems.Limelight;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Controller;

public class LimelightController extends Limelight implements Controller {

    private int state;
    TelemetryManager telemetryM;
    Follower follower;
    public LimelightController(HardwareMap hardwareMap, String name, Follower f, TelemetryManager telemetryManager){
        super(hardwareMap, name);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = f;
        state = 0;
    }

    public void setState(int stateNum){
        state = stateNum;
    }

    @Override
    public void init() {
        limelight3A.start();
    }

    //states: motif (1), relocalization(2)
    @Override
    public void update() {
        if(state == 1){
            limelight3A.pipelineSwitch(state);
            /*
            follower
            limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                if (result.isValid()) {
                    Pose3D botpose = result.getBotpose_MT2();
                    // Use botpose data
                }
            }

             */
        }
        if(state == 3){
            limelight3A.pipelineSwitch(state);
        }

        telemetryM.addData("pipeline", state);
    }

    @Override
    public void end() {

    }

    @Override
    public errors updateError() {
        return errors.RUNNING;
    }
}
