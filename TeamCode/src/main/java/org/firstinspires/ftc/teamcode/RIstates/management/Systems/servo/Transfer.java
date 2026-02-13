package org.firstinspires.ftc.teamcode.RIstates.management.Systems.servo;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.servo.CRServoBase;

public class Transfer extends CRServoBase {
    Gamepad gamepad2;
    boolean changedX;
    public Transfer(HardwareMap hardwareMap, Gamepad gamepad2){
        super(hardwareMap, "transfer");
        this.gamepad2 = gamepad2;
        changedX = false;
    }

    public void update() {
        if (gamepad2.x && !changedX) {
            changedX = true;
            run();
        } else if (!gamepad2.x && changedX) {
            changedX = false;
            stop();
        }
    }
}
