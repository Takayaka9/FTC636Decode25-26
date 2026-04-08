package org.firstinspires.ftc.teamcode.nePremier.robot.commands;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.nePremier.robot.systems.BotPose;
import org.firstinspires.ftc.teamcode.nePremier.robot.systems.Turret;
import org.firstinspires.ftc.teamcode.nePremier.robot.systems.servos.HoodServo;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.LocalizationHelper;
import org.firstinspires.ftc.teamcode.nePremier.utils.commandUtils.BaseCommand;
import org.firstinspires.ftc.teamcode.nePremier.utils.inputSystem.Control;
import org.firstinspires.ftc.teamcode.nePremier.utils.inputSystem.ControlType;
import org.firstinspires.ftc.teamcode.nePremier.utils.inputSystem.GamepadInput;

public class FuckedTurretHoodUpdate extends BaseCommand {
    private final Turret turret;
    private final HoodServo hood;
    private final Follower follower;
    private boolean overridden = false;
    private final Control weFucked;
    BotPose botPose;

    public FuckedTurretHoodUpdate(Turret turret, HoodServo hood, Follower follower, BotPose botPose, Gamepad gamepad, OhNoWeFucked ohNoWeFucked) {
        super();
        this.turret = turret;
        this.hood = hood;
        this.follower = follower;
        this.botPose = botPose;
        weFucked = new Control(GamepadInput.x, gamepad, ControlType.Hold, ohNoWeFucked);
    }

    @Override
    public void loop() {
        weFucked.update();
        if (!weFucked.isRunning()) {
            turret.trackGoal();
        }
        hood.angleHood(LocalizationHelper.getTargetDistance(botPose.getBotPose()));
    }


}
