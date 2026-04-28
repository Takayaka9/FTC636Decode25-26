package org.firstinspires.ftc.teamcode.afterPremier.opmodes.auto.variants;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.afterPremier.opmodes.auto.CloseAlliance;
import org.firstinspires.ftc.teamcode.afterPremier.util.Alliance;

@Autonomous
public class RedCloseAlliance extends CloseAlliance {
    public RedCloseAlliance(){
        super(Alliance.RED);
    }
}
