package org.firstinspires.ftc.teamcode.afterPremier.opmodes.auto;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.repeat;
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
                        race(
                                auto(),
                                waitMs(28000)
                        ),
                        PedroCommands.follow(r.f, r.createFleePath())
                )
        );
    }
    public CommandBuilder auto(){
        return sequential(
                PedroCommands.follow(r.f, p.startToShoot),
                r.autoShoot(),
                parallel(
                        PedroCommands.follow(r.f, p.intakeSpike1),
                        r.i.in()
                ),
                r.autoShoot(),
                parallel(
                        PedroCommands.follow(r.f, p.spike2andEmpty),
                        r.i.in()
                ),
                r.autoShoot(),
                repeat(
                        gateAndShoot(), 2
                )
        );
    }
    public CommandBuilder gateAndShoot(){
        return sequential(
                parallel(
                        PedroCommands.follow(r.f, p.shootToGate),
                        r.i.in()
                ),
                r.autoShoot()
        );
    }

    @Override
    public void loop() {
        r.periodic();
        r.autoLoop();
        super.loop();
    }
}
