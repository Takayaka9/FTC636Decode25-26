package org.firstinspires.ftc.teamcode.NewEnglands.utils.pedroUtils;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.alliance.Alliance;

public abstract class BasePathUpdate extends PedroUpdate implements PathUpdate {
    public BasePathUpdate(HardwareMap hardwaremap, Telemetry telemetry, Alliance alliance) {
        super(hardwaremap, telemetry, alliance);
    }

    /// MUST CALL SUPER METHOD WHEN OVERRIDING
    @Override
    public void init() {
        initDependencies();
    }

    @Override
    public void start() {
        startDependencies();
    }

    @Override
    public void update() {
        PathUpdateHelper.update(this);
        updateDependencies();
    }
}
