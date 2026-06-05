package org.firstinspires.ftc.teamcode.nePremier.opModes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.nePremier.robot.commands.TransferRun;
import org.firstinspires.ftc.teamcode.nePremier.robot.systems.TakaShooter;
import org.firstinspires.ftc.teamcode.nePremier.robot.systems.Transfer;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.LocalizationHelper;
import org.firstinspires.ftc.teamcode.nePremier.utils.inputSystem.Control;
import org.firstinspires.ftc.teamcode.nePremier.utils.inputSystem.ControlType;
import org.firstinspires.ftc.teamcode.nePremier.utils.inputSystem.GamepadInput;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Configurable
@Disabled

@TeleOp
public class ForShootingLUTShit extends OpMode {
    Follower f;
    TelemetryManager t;
    TakaShooter shooter;
    Servo h;
    Transfer transfer;
    TransferRun tRunn;
    Control runt;
    public static int tps = 0;
    public static double hPos = 1;
    @Override
    public void init() {
        f = Constants.createFollower(hardwareMap);
        f.setStartingPose(new Pose(72,72,Math.toRadians(0)));
        f.update();
        t = PanelsTelemetry.INSTANCE.getTelemetry();
        shooter = new TakaShooter(hardwareMap);
        h = hardwareMap.get(Servo.class, "hood");
        transfer = new Transfer(hardwareMap);
        tRunn = new TransferRun(transfer);
        runt = new Control(GamepadInput.right_bumper, gamepad1, ControlType.Hold, tRunn);
    }

    @Override
    public void loop() {
        f.update();
        t.update();
        shooter.test(tps);
        h.setPosition(hPos);
        t.addData("distance", LocalizationHelper.getTargetDistance(f.getPose()));
        t.addData("power", shooter.get1Power());
        t.addData("velocity", shooter.getAverageVelocity());
        runt.update();
    }
}
