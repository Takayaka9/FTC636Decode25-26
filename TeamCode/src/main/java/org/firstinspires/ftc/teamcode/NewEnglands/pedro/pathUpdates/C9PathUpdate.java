package org.firstinspires.ftc.teamcode.NewEnglands.pedro.pathUpdates;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.alliance.Alliance;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.pedroUtils.BasePathUpdate;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.pedroUtils.PathUpdateHelper;

public class C9PathUpdate extends BasePathUpdate {
    public C9PathUpdate(Alliance alliance, HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap,telemetry, alliance);
    }

    @Override
    public void autonomousPathUpdate() {
        switch(pathState) {
            //TODO: write this
        }
    }
}
