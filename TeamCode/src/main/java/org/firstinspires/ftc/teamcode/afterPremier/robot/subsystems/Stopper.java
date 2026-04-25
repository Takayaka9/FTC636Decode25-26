package org.firstinspires.ftc.teamcode.afterPremier.robot.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Configurable
public class Stopper {
    private final Servo s;
    public static double open = 0.43;
    public static double closed = 0.25;
    public Stopper(HardwareMap hardwareMap){
        s = hardwareMap.get(Servo.class, "stopper");
    }
    CommandBuilder close(){
        return instant(() -> s.setPosition(closed)).requiring(s);
    }
    CommandBuilder open(){
        return instant(() -> s.setPosition(open)).requiring(s);
    }
}
