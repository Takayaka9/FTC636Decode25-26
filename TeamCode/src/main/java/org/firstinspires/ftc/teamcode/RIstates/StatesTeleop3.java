package org.firstinspires.ftc.teamcode.RIstates;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RIstates.management.SystemManager;
import org.firstinspires.ftc.teamcode.RIstates.management.fsm.FSM;

@Configurable
@TeleOp(name = "States TeleOp 3", group = "TeleOp")
public class StatesTeleop3 extends OpMode {

    //    Subsystems + Follower
   private SystemManager manager;

    @Override
    public void loop() {
        manager.teleUpdate();
    }

    @Override
    public void init() {
        manager = new SystemManager(hardwareMap, telemetry, gamepad1, gamepad2, true);
        manager.init();
        manager.FSM.runNew(FSM.StateName.Norm);
    }

    @Override
    public void init_loop() {
        manager.telemetryM.addData("Alliance", manager.getAlliance());
        telemetry.addData("Alliance", manager.getAlliance());
    }

    @Override
    public void start() {
        manager.teleStart();
    }

    @Override
    public void stop() {
        manager.teleStop();
    }

}
