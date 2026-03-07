package org.firstinspires.ftc.teamcode.NewEnglands.utils.init;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.NewEnglands.pedroPathing.Constants2;
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
    private final TelemetryManager telemetryM;
    private final Follower follower;

    public final CommandLoop commandLoop;

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



    public Initializer(Gamepad gamepad1, Gamepad gamepad2, HardwareMap hardwareMap, Telemetry telemetry) {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants2.createFollower(hardwareMap);

        commandLoop = new CommandLoop();

        hoodServo = new HoodServo(commandLoop, hardwareMap);
//        crLiftServo = new CrLiftServo(commandLoop, hardwareMap);
        liftServo = new LiftServo(commandLoop, hardwareMap);
        stopper = new Stopper(commandLoop, hardwareMap);
        light = new Light(commandLoop, hardwareMap);
//        limelight = new Limelight(commandLoop, hardwareMap);
        shooter = new TakaShooter(commandLoop, hardwareMap);
        transfer = new Transfer(commandLoop, hardwareMap);
        turret = new Turret(commandLoop, hardwareMap, follower, gamepad2);

        intake = new Intake(commandLoop, transfer);
//        crLiftBot = new CrLiftBot(commandLoop, crLiftServo);
        liftBot = new LiftBot(commandLoop, liftServo);
//        reloc = new LLReloc(commandLoop, limelight, follower, gamepad1, telemetryM);
        makeMoves = new MakeMoves(commandLoop, follower, gamepad1);
        outake = new Outake(commandLoop, transfer);
        shoot = new Shoot(commandLoop, transfer, shooter, stopper, follower, telemetryM);
        constantShoot = new ConstantShoot(commandLoop, shooter, follower);
        turretHoodUpdate = new TurretHoodUpdate(commandLoop, turret, hoodServo, follower);
    }


}
