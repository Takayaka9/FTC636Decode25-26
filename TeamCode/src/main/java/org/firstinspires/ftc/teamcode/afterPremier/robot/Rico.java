package org.firstinspires.ftc.teamcode.afterPremier.robot;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.afterPremier.robot.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.afterPremier.robot.subsystems.Hood;
import org.firstinspires.ftc.teamcode.afterPremier.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.afterPremier.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.afterPremier.util.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Rico {
    Turret t;
    Hood h;
    Intake i;
    Flywheel fly;
    Follower f;
    public Rico(Alliance alliance, HardwareMap hardwareMap){
        f = Constants.createFollower(hardwareMap);
        t = new Turret(hardwareMap, f, alliance);

    }
}
