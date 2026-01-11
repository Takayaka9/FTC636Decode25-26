package org.firstinspires.ftc.teamcode.RIstates.management.Systems;


import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Limelight {

    Limelight3A limelight3A;

    public Limelight(HardwareMap hardwareMap, String name){
        limelight3A = hardwareMap.get(Limelight3A.class, name);
    }
//        limelight.pipelineSwitch(0);
}
