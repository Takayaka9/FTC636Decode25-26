package org.firstinspires.ftc.teamcode.robot.systems;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.commandUtils.CommandLoop;
import org.firstinspires.ftc.teamcode.utils.commandUtils.BaseSubsystem;

public class Limelight extends BaseSubsystem {
    public Limelight3A limelight3A;

    public Limelight(CommandLoop maps, HardwareMap hardwareMap) {
        super(maps);
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
    }

    public void switchPipeline(int pipeline){
        limelight3A.pipelineSwitch(pipeline);
    }

}
