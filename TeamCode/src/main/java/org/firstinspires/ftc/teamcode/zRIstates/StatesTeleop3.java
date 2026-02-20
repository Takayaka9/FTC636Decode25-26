package org.firstinspires.ftc.teamcode.zRIstates;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.zRIstates.management.SystemManager;
import org.firstinspires.ftc.teamcode.zRIstates.management.fsm.FSM;

import java.util.List;

@Configurable
@TeleOp(name = "States TeleOp 3", group = "TeleOp")
public class StatesTeleop3 extends OpMode {

    //    Subsystems + Follower
   private SystemManager manager;
   List<LynxModule> allHubs;

    @Override
    public void loop() {
        manager.teleUpdate();
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    @Override
    public void init() {
        manager = new SystemManager(hardwareMap, telemetry, gamepad1, gamepad2, true, false);
        manager.init();
        manager.FSM.runNew(FSM.StateName.Norm);
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
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
