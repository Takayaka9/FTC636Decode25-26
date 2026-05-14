package org.firstinspires.ftc.teamcode.afterPremier.opmodes.auto;

import static com.pedropathing.ivy.commands.Commands.lazy;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.repeat;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.ivy.pedro.PedroCommands;

import org.firstinspires.ftc.teamcode.afterPremier.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.afterPremier.robot.Rico;
import org.firstinspires.ftc.teamcode.afterPremier.util.Alliance;
import org.firstinspires.ftc.teamcode.afterPremier.util.pathing.FarPaths;

public class Far extends BaseOpMode {
    FarPaths f;
    Rico r;
    Alliance a;
    public Far(Alliance a){
        super();
        this.a = a;
    }
    @Override
    public void init() {
        super.init();
        r = new Rico(hardwareMap, a);
    }

    @Override
    public void start() {
        super.start();
        r.f.setStartingPose(f.getStartingPose());
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
    public static int farIntake = 2;
    public CommandBuilder auto(){
        return sequential(
                PedroCommands.follow(r.f, f.startToShoot),
                r.autoShoot(),
                parallel(
                        PedroCommands.follow(r.f, f.intakeSpike3, 0.75),
                        r.i.in()
                ),
                PedroCommands.follow(r.f, f.spike3toShoot),
                r.autoShoot(),
                parallel(
                        PedroCommands.follow(r.f, f.shootToWall),
                        r.i.in()
                ),
                PedroCommands.follow(r.f, f.wallToShoot),
                r.autoShoot(),
                repeat(
                        farIntake(), farIntake
                )
        );
    }
    public CommandBuilder farIntake(){
        return sequential(
                parallel(
                        PedroCommands.follow(r.f, f.shootToFarIntake),
                        r.i.in()
                ),
                PedroCommands.follow(r.f, f.farIntakeToShoot),
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
