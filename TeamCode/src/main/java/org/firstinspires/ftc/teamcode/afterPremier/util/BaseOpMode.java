package org.firstinspires.ftc.teamcode.afterPremier.util;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class BaseOpMode extends OpMode {
    @Override
    public void init() {
        Scheduler.reset();
    }

    @Override
    public void loop() {
        Scheduler.execute();
    }
    public void schedule(Command...commands){
        Scheduler.schedule(commands);
    }
}
