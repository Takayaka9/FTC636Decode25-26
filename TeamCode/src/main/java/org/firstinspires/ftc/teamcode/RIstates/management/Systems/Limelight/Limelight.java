package org.firstinspires.ftc.teamcode.RIstates.management.Systems.Limelight;


import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Limelight {
    Limelight3A limelight3A;

    public Limelight(HardwareMap hardwareMap, String name){
        limelight3A = hardwareMap.get(Limelight3A.class, name);
    }

    public void start(){
        limelight3A.start();
    }

    public void stop(){
        limelight3A.stop();
    }

    public void switchPipeline(int pipeline){
        limelight3A.pipelineSwitch(pipeline);
    }

//        limelight.pipelineSwitch(0);
    //3 is apriltag loc
}
