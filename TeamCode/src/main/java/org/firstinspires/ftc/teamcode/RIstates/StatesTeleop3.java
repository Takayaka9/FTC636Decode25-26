package org.firstinspires.ftc.teamcode.RIstates;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RIstates.management.SystemManager;

@Configurable
@TeleOp(name = "States TeleOp 3", group = "TeleOp")
public class StatesTeleop3 extends OpMode {

    //    Subsystems + Follower
    SystemManager manager;
    HardwareMap hardwareMap;


    @Override
    public void loop() {
        manager.teleUpdate();
    }

    @Override
    public void init() {
        manager = new SystemManager(hardwareMap, gamepad1, gamepad2, true);
        manager.init();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        manager.teleStart();
    }

    @Override
    public void stop() {
        manager.teleStop();
    }

}
