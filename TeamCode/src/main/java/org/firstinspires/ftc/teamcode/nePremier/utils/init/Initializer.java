package org.firstinspires.ftc.teamcode.nePremier.utils.init;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.nePremier.robot.commands.NewCRLift;
import org.firstinspires.ftc.teamcode.nePremier.robot.commands.NewCRLiftDown;
import org.firstinspires.ftc.teamcode.nePremier.robot.commands.TransferRun;
import org.firstinspires.ftc.teamcode.nePremier.robot.systems.servos.NewCRLiftServo;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants2;
import org.firstinspires.ftc.teamcode.nePremier.robot.commands.AllianceBlue;
import org.firstinspires.ftc.teamcode.nePremier.robot.commands.AllianceRed;
import org.firstinspires.ftc.teamcode.nePremier.robot.commands.ConstantFlywheelSpin;
import org.firstinspires.ftc.teamcode.nePremier.robot.commands.LiftBot;
import org.firstinspires.ftc.teamcode.nePremier.robot.commands.MakeMoves;
import org.firstinspires.ftc.teamcode.nePremier.robot.commands.OhNoWeFucked;
import org.firstinspires.ftc.teamcode.nePremier.robot.commands.Outake;
import org.firstinspires.ftc.teamcode.nePremier.robot.commands.ResetForTele;
import org.firstinspires.ftc.teamcode.nePremier.robot.commands.Shoot;
import org.firstinspires.ftc.teamcode.nePremier.robot.commands.TurretHoodUpdate;
import org.firstinspires.ftc.teamcode.nePremier.robot.systems.Light;
import org.firstinspires.ftc.teamcode.nePremier.robot.systems.TakaShooter;
import org.firstinspires.ftc.teamcode.nePremier.robot.systems.Transfer;
import org.firstinspires.ftc.teamcode.nePremier.robot.systems.Turret;
import org.firstinspires.ftc.teamcode.nePremier.robot.systems.servos.HoodServo;
import org.firstinspires.ftc.teamcode.nePremier.robot.systems.servos.LiftServo;
import org.firstinspires.ftc.teamcode.nePremier.robot.systems.servos.Stopper;

public class Initializer {

    //dependencies
    public final TelemetryManager telemetryM;
    public final Follower follower;


    public final HoodServo hoodServo;
//    public final CrLiftServo crLiftServo;
//    public final LiftServo liftServo;
    public final Stopper stopper;
    public final Light light;
//    public final Limelight limelight;
    public final TakaShooter shooter;
    public final Transfer transfer;
    public final Turret turret;
    public final NewCRLiftServo liftServo;


    public final TransferRun transferRun;
//    public final CrLiftBot crLiftBot;
    public final NewCRLift liftBot;
    public final NewCRLiftDown liftDown;
//    public final LLReloc reloc;
    public final MakeMoves makeMoves;
    public final Outake outake;
    public final Shoot shoot;
    public final ConstantFlywheelSpin constantFlywheelSpin;
    public final TurretHoodUpdate turretHoodUpdate;
    public final AllianceBlue blue;
    public final AllianceRed red;
    public final ResetForTele resetForTele;
    public final OhNoWeFucked ohNoWeFucked;





    public Initializer(Gamepad gamepad1, Gamepad gamepad2, HardwareMap hardwareMap, Telemetry ignoredTelemetry) {
        /// lil stuff
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants2.createFollower(hardwareMap);


        /// Subsystems
        hoodServo = new HoodServo(hardwareMap);
//        crLiftServo = new CrLiftServo(hardwareMap);
        liftServo = new NewCRLiftServo(hardwareMap);
        stopper = new Stopper(hardwareMap);
        light = new Light(hardwareMap);
//        limelight = new Limelight(hardwareMap);
        shooter = new TakaShooter(hardwareMap);
        transfer = new Transfer(hardwareMap);
        turret = new Turret(hardwareMap, follower);


        ///commands
        transferRun = new TransferRun(transfer);
//        crLiftBot = new CrLiftBot(crLiftServo);
        liftBot = new NewCRLift(liftServo);
        liftDown = new NewCRLiftDown(liftServo);
//        reloc = new LLReloc(limelight, follower, gamepad1, telemetryM);
        makeMoves = new MakeMoves(follower, gamepad1);
        outake = new Outake(transfer);
        shoot = new Shoot(transfer, shooter, stopper);
        constantFlywheelSpin = new ConstantFlywheelSpin(shooter, follower);
        turretHoodUpdate = new TurretHoodUpdate(turret, hoodServo, follower, gamepad2);
        blue = new AllianceBlue();
        red = new AllianceRed();
        resetForTele = new ResetForTele(turret);
        ohNoWeFucked = new OhNoWeFucked(turret, gamepad2);

    }


}

