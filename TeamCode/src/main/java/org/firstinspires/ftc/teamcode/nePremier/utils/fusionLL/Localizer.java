package org.firstinspires.ftc.teamcode.nePremier.utils.fusionLL;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.nePremier.robot.systems.Light;
import org.firstinspires.ftc.teamcode.nePremier.utils.commandUtils.BaseCommand;

public class Localizer extends BaseCommand implements Localize {
    private final Limelight3A ll;
    private final Follower follower;
    private final Light light;
    private final TelemetryManager telemetryM;

    public Localizer(HardwareMap hardwareMap, Follower follower, Light light) {
        ll = hardwareMap.get(Limelight3A.class, "limelight");
        this.follower = follower;
        this.light = light;
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        ll.start();
        ll.pipelineSwitch(LocalizerConfig.pipeline);
    }

    @Override
    public void loop() {
        Pose nPose = localize();
        if (nPose != null) {
            if (compare(nPose)) {
                follower.setPose(nPose);
                light.green();
            } else {
                light.blue();
            }
        } else {
            light.red();
        }

    }

    @Override
    public Pose localize() {
        return MT2Helper.loop(ll, follower, telemetryM);
    }

    @Override
    public boolean compare(Pose nPose) {
        //gets int rounded poses from follower
        int cX = (int) Math.round(follower.getPose().getX());
        int cY = (int) Math.round(follower.getPose().getY());

        //gets data from new pose
        int nX = (int) Math.round(nPose.getX());
        int nY = (int) Math.round(nPose.getY());

        //gets x and y distances squared
        int x = Math.abs(cX - nX) * Math.abs(cX - nX);
        int y = Math.abs(cY - nY) * Math.abs(cY - nY);

        //gets distance
        double distance = Math.sqrt(x + y);

        if (distance < LocalizerConfig.maxDist) {
            follower.setPose(nPose);
            return true;
        }
        return false;
    }


}
