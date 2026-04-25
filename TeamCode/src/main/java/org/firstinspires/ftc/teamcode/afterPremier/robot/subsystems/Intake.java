package org.firstinspires.ftc.teamcode.afterPremier.robot.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Configurable
public class Intake {
    private final DcMotorEx i;
    public Intake(HardwareMap hardwareMap){
        i = hardwareMap.get(DcMotorEx.class, "transfer");
    }
    CommandBuilder in(){
        return instant(() -> i.setPower(1)).requiring(i);
    }
    CommandBuilder out(){
        return instant(() -> i.setPower(-1)).requiring(i);
    }
    CommandBuilder off(){
        return instant(() -> i.setPower(0)).requiring(i);
    }
}
