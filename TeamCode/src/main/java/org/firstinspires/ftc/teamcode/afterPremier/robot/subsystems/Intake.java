package org.firstinspires.ftc.teamcode.afterPremier.robot.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Configurable
public class Intake {
    private final DcMotorEx i;
    public Intake(HardwareMap hardwareMap){
        i = hardwareMap.get(DcMotorEx.class, "transfer");
    }
    public void in(){
        i.setPower(1);
    }
    public void out(){
        i.setPower(-1);
    }
    public void off(){
        i.setPower(0);
    }
}
