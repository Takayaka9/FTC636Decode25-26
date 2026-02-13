package org.firstinspires.ftc.teamcode.utils.init;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants2;
import org.firstinspires.ftc.teamcode.utils.alliance.Alliance;

public abstract class Initializer {

    //dependencies
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    private final Telemetry telemetry;
    private final TelemetryManager telemetryM;
    private final Follower follower;

    public Initializer(Gamepad gamepad1, Gamepad gamepad2, HardwareMap hardwareMap, Telemetry telemetry, Alliance alliance) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants2.createFollower(hardwareMap);
    }
}
