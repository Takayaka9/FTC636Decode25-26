package org.firstinspires.ftc.teamcode.NewEnglands.utils.pedroUtils;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.init.Initializer;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.alliance.Alliance;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.pedroUtils.interfaceUtils.PathUpdate;

public class PedroLoops extends Initializer {

    // plan is to create a pedroloops object and then run its methods
    // the selected pathupdate class will handle most of the work
    public PedroLoops(Gamepad gamepad1, Gamepad gamepad2, HardwareMap hardwareMap, Telemetry telemetry, PathUpdate pathUpdate, Alliance alliance) {
        super(gamepad1, gamepad2, hardwareMap, telemetry, alliance);
    }

    public void init() {

    }

    public void init_loop() {
    }

    public void start() {

    }

    public void loop() {

    }

    public void stop() {
    }

}
