package org.firstinspires.ftc.teamcode.nePremier.opModes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.nePremier.robot.systems.TakaShooter;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.Alliance;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.TDistHelper;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Configurable
@TeleOp
public class ForShootingLUTShit extends OpMode {
    Follower f;
    TelemetryManager t;
    TakaShooter shooter;
    Servo h;
    public static int tps = 0;
    public static double hPos = 1;
    @Override
    public void init() {
        f = Constants.createFollower(hardwareMap);
        f.update();
        t = PanelsTelemetry.INSTANCE.getTelemetry();
        shooter = new TakaShooter(hardwareMap);
        h = hardwareMap.get(Servo.class, "hood");
    }

    @Override
    public void loop() {
        f.update();
        t.update();

        shooter.test(tps);
        h.setPosition(hPos);
        t.addData("distance", TDistHelper.getTargetDistance(f.getPose(), Alliance.RED));
    }
}
