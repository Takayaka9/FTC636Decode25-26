package org.firstinspires.ftc.teamcode.afterPremier.opmodes.auto;

import static com.pedropathing.ivy.commands.Commands.lazy;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.repeat;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.pedro.PedroCommands;

import org.firstinspires.ftc.teamcode.afterPremier.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.afterPremier.robot.Rico;
import org.firstinspires.ftc.teamcode.afterPremier.util.Alliance;
import org.firstinspires.ftc.teamcode.afterPremier.util.pathing.ClosePaths;
@Configurable
public class CloseSolo extends BaseOpMode {
    ClosePaths p;
    Rico r;
    Alliance a;
    public CloseSolo(Alliance a){
        super();
        this.a = a;

    }
    @Override
    public void init() {
        super.init();
        r = new Rico(hardwareMap, a);
        p = new ClosePaths(a, r.f);
    }

    @Override
    public void start() {
        super.start();
        r.f.setStartingPose(p.getStartingPose());
        r.s.close().schedule();
        schedule(
                sequential(
                        race(
                                auto(),
                                waitMs(28000)
                        ),
                        lazy(() -> PedroCommands.follow(r.f, r.createFleePath()))
                )
        );
    }
    public static int gates = 2;
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
                        PedroCommands.follow(r.f, p.intakeSpike2),
                        r.i.in()
                ),
                r.autoShoot(),
                repeat(
                        gateAndShoot(), gates
                ),
                parallel(
                        PedroCommands.follow(r.f, p.intakeSpike3),
                        r.i.in()
                ),
                r.autoShoot()
        );
    }
    public static int gateWait = 1400;
    public CommandBuilder gateAndShoot(){
        return sequential(
                parallel(
                        PedroCommands.follow(r.f, p.shootToGate),
                        r.i.in()
                ),
                waitMs(gateWait),
                PedroCommands.follow(r.f, p.gateToShoot),
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
