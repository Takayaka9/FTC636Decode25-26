package org.firstinspires.ftc.teamcode.NewEnglands.robot.commands;

import org.firstinspires.ftc.teamcode.NewEnglands.robot.systems.Turret;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.BaseCommand;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.inputSystem.Control;

public class ResetForTele extends BaseCommand {
    Turret turret;
//    Control[] controls;
    public ResetForTele (Turret turret, Control... controlsToStop) {
        this.turret = turret;
//        for (int i = 0; i < controlsToStop.length;) {
//            controls[i] = controlsToStop[i];
//            i++;
//        }
    }

    @Override
    public void init() {
//        for (int i = 0; i < controls.length;) {
//            controls[i].stop();
//        }
    }

    @Override
    public void loop() {
        turret.turnTurret(0);
    }
}
