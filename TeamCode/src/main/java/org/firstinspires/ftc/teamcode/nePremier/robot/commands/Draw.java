package org.firstinspires.ftc.teamcode.nePremier.robot.commands;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.nePremier.utils.commandUtils.BaseCommand;
import org.firstinspires.ftc.teamcode.nePremier.utils.pedroUtils.Drawing;

public class Draw extends BaseCommand {
    private final Follower follower;

    public Draw(Follower follower) {
        this.follower = follower;
    }

    @Override
    public void init() {
        Drawing.init();
    }

    @Override
    public void loop() {
        Drawing.drawDebug(follower);
    }
}
