package org.firstinspires.ftc.teamcode.nePremier.robot.commands;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.Alliance;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.CurrentAlliance;
import org.firstinspires.ftc.teamcode.nePremier.utils.commandUtils.BaseCommand;

@Configurable
public class SimpleLoc extends BaseCommand {
    private final Follower f;
    private final Pose resetPose = new Pose(135.4, 8.5, 0);

    public SimpleLoc(Follower follower) {
        super();
        this.f = follower;
    }

    @Override
    public void init() {
        if (CurrentAlliance.alliance == Alliance.BLUE) f.setPose(resetPose);
        else if (CurrentAlliance.alliance == Alliance.RED) f.setPose(resetPose.mirror());
    }

}
