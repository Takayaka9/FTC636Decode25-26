package org.firstinspires.ftc.teamcode.NewEnglands.robot.systems.servos;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.BaseSubsystem;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.CommandLoop;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.servo.GamepadServoImplEx;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.servo.ServoImplExBase;

public class LiftServo extends BaseSubsystem {
    private static class LiftPositions {
        public static double upR = 0;
        public static double upL = 1;
        public static double downR = 1;
        public static double downL = 0;
    }
    private final ServoImplEx l;
    private final ServoImplEx r;

    public LiftServo(CommandLoop maps, HardwareMap hardwareMap) {
        super(maps);
        l = hardwareMap.get(ServoImplEx.class, "lift1");
        r = hardwareMap.get(ServoImplEx.class, "lift2");
    }

    public void up() {
        l.setPosition(LiftPositions.upL);
        r.setPosition(LiftPositions.upR);
    }

    public void down() {
        l.setPosition(LiftPositions.downL);
        r.setPosition(LiftPositions.downR);
    }
}
