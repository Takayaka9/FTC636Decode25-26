package org.firstinspires.ftc.teamcode.nePremier.opModes.tests;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.LocalizationHelper;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.nePremier.robot.systems.TakaShooter;
import org.firstinspires.ftc.teamcode.nePremier.utils.alliance.Alliance;
import org.firstinspires.ftc.teamcode.nePremier.utils.init.Initializer;



//TODO: FIX TO NOT USE COMMAND LOOP IF WE NEED THIS
@Configurable
@Disabled

@TeleOp
public class LUTTester extends OpMode {
    Initializer initializer = null;
    Alliance alliance = null;
    InterpLUT lut = null;
    TakaShooter shooter = null;
    LocalizationHelper distance = null;
    Follower follower = null;
    @Override
    public void init() {
        initializer = new Initializer(gamepad1, gamepad2, hardwareMap, telemetry);
        follower = Constants.createFollower(hardwareMap);
    }

    @Override
    public void loop() {
        follower.update();

//        TODO: EVEN WITH NEW SYSTEM THE COMMANDS WOULD BE CALLED INCORRECTLY
//        TODO: CHANGE LOGIC SO THAT THEY ARE ONLY CALLED ONCE
//        if(gamepad1.right_bumper){
//            initializer.commandLoop.runCommand(initializer.transferRun);
//        }
//        else{
//            initializer.commandLoop.stopCommand(initializer.transferRun);
//        }
        double d1 = 36;  double r1 = 900;
        double d2 = 50;  double r2 = 900;
        double d3 = 75;  double r3 = 1000;
        double d4 = 96;  double r4 = 1150;
        double d5 = 108;  double r5 = 1400;
        double d6 = 150;  double r6 = 1400;
        lut.add(0, r1);
        lut.add(d1, r1);
        lut.add(d2, r2);
        lut.add(d3, r3);
        lut.add(d4, r4);
        lut.add(d5, r5);
        lut.add(d6, r6);
        lut.add(1000, r6);
        lut.createLUT();
//        if(gamepad1.a){
//            shooter.updateLeft(lut.get(distance.getTargetDistance(follower.getPose(), initializer.
    }
}
