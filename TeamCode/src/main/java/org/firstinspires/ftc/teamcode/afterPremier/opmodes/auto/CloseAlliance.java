package org.firstinspires.ftc.teamcode.afterPremier.opmodes.auto;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.pedro.PedroCommands;

import org.firstinspires.ftc.teamcode.afterPremier.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.afterPremier.robot.Rico;
import org.firstinspires.ftc.teamcode.afterPremier.util.Alliance;
import org.firstinspires.ftc.teamcode.afterPremier.util.pathing.ClosePaths;

public class CloseAlliance extends BaseOpMode {
    ClosePaths p;
    Rico r;
    public CloseAlliance(Alliance a){
        super();
        r = new Rico(hardwareMap, a);
        p = new ClosePaths(a, r.f);
    }
    @Override
    public void init() {
        super.init();
    }

    @Override
    public void start() {
        super.start();
        r.s.close();
        schedule(
                sequential(
                        PedroCommands.follow(r.f, p.startToShoot),
                        r.shoot(),
                        parallel(
                                PedroCommands.follow(r.f, p.intakeSpike1),
                                r.i.in()
                        ),
                        r.shoot(),
                        parallel(
                                PedroCommands.follow(r.f, p.spike2andEmpty),
                                r.i.in()
                        ),
                        r.shoot()
                )
        );
    }

    @Override
    public void loop() {
        r.periodic();
        r.autoLoop();
        super.loop();
    }
}
