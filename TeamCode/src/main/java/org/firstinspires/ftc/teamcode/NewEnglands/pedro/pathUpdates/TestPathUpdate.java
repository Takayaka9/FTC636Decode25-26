package org.firstinspires.ftc.teamcode.NewEnglands.pedro.pathUpdates;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.alliance.Alliance;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.pedroUtils.BasePathUpdate;

public class TestPathUpdate extends BasePathUpdate {

    public TestPathUpdate(HardwareMap hardwareMap, Telemetry telemetry, Alliance alliance) {
        super(hardwareMap, telemetry, alliance);
    }
    @Override
    public void autonomousPathUpdate() {}

}
