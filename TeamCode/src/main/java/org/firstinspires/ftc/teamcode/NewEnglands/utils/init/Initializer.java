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
import org.firstinspires.ftc.teamcode.NewEnglands.robot.commands.ConstantShoot;
import org.firstinspires.ftc.teamcode.NewEnglands.robot.commands.Intake;
import org.firstinspires.ftc.teamcode.NewEnglands.robot.commands.LLReloc;
import org.firstinspires.ftc.teamcode.NewEnglands.robot.commands.CrLiftBot;
import org.firstinspires.ftc.teamcode.NewEnglands.robot.commands.LiftBot;
import org.firstinspires.ftc.teamcode.NewEnglands.robot.commands.MakeMoves;
import org.firstinspires.ftc.teamcode.NewEnglands.robot.commands.Outake;
import org.firstinspires.ftc.teamcode.NewEnglands.robot.commands.Shoot;
import org.firstinspires.ftc.teamcode.NewEnglands.robot.commands.TurretHoodUpdate;
import org.firstinspires.ftc.teamcode.NewEnglands.robot.systems.Light;
import org.firstinspires.ftc.teamcode.NewEnglands.robot.systems.Limelight;
import org.firstinspires.ftc.teamcode.NewEnglands.robot.systems.TakaShooter;
import org.firstinspires.ftc.teamcode.NewEnglands.robot.systems.Transfer;
import org.firstinspires.ftc.teamcode.NewEnglands.robot.systems.Turret;
import org.firstinspires.ftc.teamcode.NewEnglands.robot.systems.servos.HoodServo;
import org.firstinspires.ftc.teamcode.NewEnglands.robot.systems.servos.CrLiftServo;
import org.firstinspires.ftc.teamcode.NewEnglands.robot.systems.servos.LiftServo;
import org.firstinspires.ftc.teamcode.NewEnglands.robot.systems.servos.Stopper;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.CommandLoop;

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
    public final ConstantShoot constantShoot;
    public final TurretHoodUpdate turretHoodUpdate;
    public final AllianceBlue blue;
    public final AllianceRed red;



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
        constantShoot = new ConstantShoot(shooter, follower);
        turretHoodUpdate = new TurretHoodUpdate(turret, hoodServo, follower);
        blue = new AllianceBlue();
        red = new AllianceRed();

    }


}

