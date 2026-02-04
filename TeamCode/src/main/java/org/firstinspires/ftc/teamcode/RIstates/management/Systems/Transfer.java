package org.firstinspires.ftc.teamcode.RIstates.management.Systems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Transfer {
    CRServoImpl transfer;
    public Transfer(HardwareMap hardwareMap){
        transfer = hardwareMap.get(CRServoImpl.class, "transfer");
    }
    public static double closePower = 1;
    public void runCLose(){
        transfer.setPower(closePower);
    }
    public static double farPower = 1;
    public void runFar(){
        transfer.setPower(farPower);
    }
    public void stop(){
        transfer.setPower(0);
    }
}
