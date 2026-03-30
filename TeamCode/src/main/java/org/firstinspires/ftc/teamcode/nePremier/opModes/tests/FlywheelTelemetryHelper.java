package org.firstinspires.ftc.teamcode.nePremier.opModes.tests;

import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.CurrentAlliance;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.LocalizationHelper;
import org.firstinspires.ftc.teamcode.nePremier.utils.init.Initializer;

public class FlywheelTelemetryHelper {
    public static void loop(Initializer i) {
        i.telemetryM.update();
        i.telemetryM.addData(
                "target distance - 1ft = 12u (pedro pose unit)",
                LocalizationHelper.getTargetDistance(i.follower.getPose(), CurrentAlliance.alliance)
        );
        i.telemetryM.addData(
                "alliance",
                CurrentAlliance.alliance
        );

        //shooter data
        i.telemetryM.addData(
                "target TPS - 2800 = 6000",
                i.shooter.getShooterTPS(LocalizationHelper.getTargetDistance(i.follower.getPose(), CurrentAlliance.alliance))
        );
        i.telemetryM.addData(
                "1 output power (0-1)",
                i.shooter.get1Power()
        );
        i.telemetryM.addData(
                "2 output power (0-1)",
                i.shooter.get2Power()
        );
        i.telemetryM.addData(
                "2 velocity in tps",
                i.shooter.getShooter2tps()
        );
        i.telemetryM.addData(
                "1 velocity in tps",
                i.shooter.getShooter1tps()
        );
        i.telemetryM.addData(
                "average velocity in tps",
                i.shooter.getAverageVelocity()
        );
        i.telemetryM.addData(
                "error",
                i.shooter.getError()
        );
        i.telemetryM.addData(
                "kpON",
                i.shooter.getKpOn()
        );
    }
}
