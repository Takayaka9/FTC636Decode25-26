package org.firstinspires.ftc.teamcode.nePremier.robot.commands;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.nePremier.robot.systems.Turret;
import org.firstinspires.ftc.teamcode.nePremier.robot.systems.servos.HoodServo;
import org.firstinspires.ftc.teamcode.nePremier.robot.systems.turret.TurretI;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.CurrentAlliance;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.TDistHelper;
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


    public FuckedTurretHoodUpdate(TurretI turret, HoodServo hood, Follower follower, Gamepad gamepad, OhNoWeFucked ohNoWeFucked) {
        super();
        this.turret = turret;
        this.hood = hood;
        this.follower = follower;
        weFucked = new Control(GamepadInput.x, gamepad, ControlType.Hold, ohNoWeFucked);
    }

    @Override
    public void loop() {
        weFucked.update();
        if (!weFucked.isRunning()) {
            turret.trackGoal();
        }
        hood.angleHood(TDistHelper.getTargetDistance(follower.getPose(), CurrentAlliance.alliance));
    }


}
