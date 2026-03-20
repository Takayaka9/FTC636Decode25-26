package org.firstinspires.ftc.teamcode.nePremier.robot.systems;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.nePremier.utils.commandUtils.CommandLoop;
import org.firstinspires.ftc.teamcode.nePremier.utils.commandUtils.BaseSubsystem;

@Deprecated
public class Limelight extends BaseSubsystem {
    public final Limelight3A ll;

    public Limelight(CommandLoop maps, HardwareMap hardwareMap) {
        super();
        ll = hardwareMap.get(Limelight3A.class, "limelight");

        //init ll and switches pipeline to 3.
    }
}
