package org.firstinspires.ftc.teamcode.afterPremier.robot.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.afterPremier.util.Alliance;

@Configurable
public class Turret {
    private final DcMotorEx t;
    Follower f;
    Alliance a;
    public Turret(HardwareMap hardwareMap, Follower follower, Alliance alliance){
        t = hardwareMap.get(DcMotorEx.class, "turret");
        t.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        t.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        f = follower;
        a = alliance;
    }

}
