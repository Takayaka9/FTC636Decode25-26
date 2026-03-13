package org.firstinspires.ftc.teamcode.NewEnglands.utils.init;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.NewEnglands.pedroPathing.Constants2;
import org.firstinspires.ftc.teamcode.NewEnglands.robot.commands.AllianceBlue;
import org.firstinspires.ftc.teamcode.NewEnglands.robot.commands.AllianceRed;
import org.firstinspires.ftc.teamcode.NewEnglands.robot.commands.ConstantFlywheelSpin;
import org.firstinspires.ftc.teamcode.NewEnglands.robot.commands.Intake;
import org.firstinspires.ftc.teamcode.NewEnglands.robot.commands.LiftBot;
import org.firstinspires.ftc.teamcode.NewEnglands.robot.commands.MakeMoves;
import org.firstinspires.ftc.teamcode.NewEnglands.robot.commands.OhNoWeFucked;
import org.firstinspires.ftc.teamcode.NewEnglands.robot.commands.Outake;
import org.firstinspires.ftc.teamcode.NewEnglands.robot.commands.ResetForTele;
import org.firstinspires.ftc.teamcode.NewEnglands.robot.commands.Shoot;
import org.firstinspires.ftc.teamcode.NewEnglands.robot.commands.TurretHoodUpdate;
import org.firstinspires.ftc.teamcode.NewEnglands.robot.systems.Light;
import org.firstinspires.ftc.teamcode.NewEnglands.robot.systems.TakaShooter;
import org.firstinspires.ftc.teamcode.NewEnglands.robot.systems.Transfer;
import org.firstinspires.ftc.teamcode.NewEnglands.robot.systems.Turret;
import org.firstinspires.ftc.teamcode.NewEnglands.robot.systems.servos.HoodServo;
import org.firstinspires.ftc.teamcode.NewEnglands.robot.systems.servos.LiftServo;
import org.firstinspires.ftc.teamcode.NewEnglands.robot.systems.servos.Stopper;

public class Initializer {

    //dependencies
    public final TelemetryManager telemetryM;
    public final Follower follower;


    public final HoodServo hoodServo;
//    public final CrLiftServo crLiftServo;
    public final LiftServo liftServo;
    public final Stopper stopper;
    public final Light light;
//    public final Limelight limelight;
    public final TakaShooter shooter;
    public final Transfer transfer;
    public final Turret turret;


    public final Intake intake;
//    public final CrLiftBot crLiftBot;
    public final LiftBot liftBot;
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





    public Initializer(Gamepad gamepad1, Gamepad gamepad2, HardwareMap hardwareMap, Telemetry telemetry) {
        /// lil stuff
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants2.createFollower(hardwareMap);


        /// Subsystems
        hoodServo = new HoodServo(hardwareMap);
//        crLiftServo = new CrLiftServo(hardwareMap);
        liftServo = new LiftServo(hardwareMap);
        stopper = new Stopper(hardwareMap);
        light = new Light(hardwareMap);
//        limelight = new Limelight(hardwareMap);
        shooter = new TakaShooter(hardwareMap);
        transfer = new Transfer(hardwareMap);
        turret = new Turret(hardwareMap, follower, gamepad2);


        ///commands
        intake = new Intake(transfer);
//        crLiftBot = new CrLiftBot(crLiftServo);
        liftBot = new LiftBot(liftServo);
//        reloc = new LLReloc(limelight, follower, gamepad1, telemetryM);
        makeMoves = new MakeMoves(follower, gamepad1);
        outake = new Outake(transfer);
        shoot = new Shoot(transfer, shooter, stopper, follower, telemetryM);
        constantFlywheelSpin = new ConstantFlywheelSpin(shooter, follower);
        turretHoodUpdate = new TurretHoodUpdate(turret, hoodServo, follower, gamepad2);
        blue = new AllianceBlue();
        red = new AllianceRed();
        resetForTele = new ResetForTele(turret);
        ohNoWeFucked = new OhNoWeFucked(turret, gamepad2);

    }


}

