package org.firstinspires.ftc.teamcode.RIstates.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RIstates.management.SystemManager;
import org.firstinspires.ftc.teamcode.RIstates.management.fsm.FSM;

@Disabled
@Configurable
@TeleOp(name = "Turret Tester", group = "TeleOp")
public class AimingTester extends OpMode {

    /// set in panels (pedro units, 12u = 1ft)
    private static double tps = 0;
    private SystemManager manager;

    private static int targetPosition = 0;
    @Override
    public void init() {
        manager = new SystemManager(hardwareMap, telemetry, gamepad1, gamepad2, true, true);
        manager.init();
        manager.FSM.runNew(FSM.StateName.AllianceSelect);
    }
    @Override
    public void init_loop() {
        manager.telemetryM.addData("Alliance", manager.getAlliance());
        telemetry.addData("Alliance", manager.getAlliance());
    }
    @Override
    public void loop() {
        manager.testUpdate();
        manager.shooter.test(tps);

        while(gamepad1.x && !gamepad1.b && !gamepad1.a) {
            manager.turret.trackGoal(1);
        }

        while (gamepad1.b && !gamepad1.x && !gamepad1.a) {
            manager.turret.trackGoal(2);
        }
        while (gamepad1.a && !gamepad1.x && !gamepad1.b) {
            manager.turret.turnTurret(targetPosition);
        }

    }
}
