package org.firstinspires.ftc.teamcode.scrims;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//pray
@Configurable
@Autonomous(name = "ScrimAuto", group = "Autonomous")
public class ScrimAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
    }
}
