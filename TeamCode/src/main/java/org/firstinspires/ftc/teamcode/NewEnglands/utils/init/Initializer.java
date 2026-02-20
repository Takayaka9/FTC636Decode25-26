package org.firstinspires.ftc.teamcode.NewEnglands.utils.init;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.NewEnglands.pedroPathing.Constants2;
import org.firstinspires.ftc.teamcode.NewEnglands.robot.commands.Intake;
import org.firstinspires.ftc.teamcode.NewEnglands.robot.commands.LLReloc;
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
import org.firstinspires.ftc.teamcode.NewEnglands.robot.systems.servos.LiftServo;
import org.firstinspires.ftc.teamcode.NewEnglands.robot.systems.servos.Stopper;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.alliance.Alliance;
import org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils.CommandLoop;

public abstract class Initializer {

    //dependencies
    public final TelemetryManager telemetryM;
    public final Follower follower;

    public final CommandLoop commandLoop;

    private final HoodServo hoodServo;
    private final LiftServo liftServo;
    private final Stopper stopper;
    private final Light light;
    private final Limelight limelight;
    private final TakaShooter shooter;
    private final Transfer transfer;
    private final Turret turret;

    private final Intake intake;
    private final LiftBot liftBot;
    private final LLReloc reloc;
    private final MakeMoves makeMoves;
    private final Outake outake;
    private final Shoot shoot;
    private final TurretHoodUpdate turretHoodUpdate;



    public Initializer(Gamepad gamepad1, Gamepad gamepad2, HardwareMap hardwareMap, Telemetry telemetry, Alliance alliance) {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants2.createFollower(hardwareMap);

        commandLoop = new CommandLoop();

        hoodServo = new HoodServo(commandLoop, hardwareMap);
        liftServo = new LiftServo(commandLoop, hardwareMap);
        stopper = new Stopper(commandLoop, hardwareMap);
        light = new Light(commandLoop, hardwareMap);
        limelight = new Limelight(commandLoop, hardwareMap);
        shooter = new TakaShooter(commandLoop, hardwareMap);
        transfer = new Transfer(commandLoop, hardwareMap);
        turret = new Turret(commandLoop, hardwareMap, follower, gamepad2);

        intake = new Intake(commandLoop, transfer);
        liftBot = new LiftBot(commandLoop,liftServo);
        reloc = new LLReloc(commandLoop, limelight, follower, telemetryM);
        makeMoves = new MakeMoves(commandLoop, follower, gamepad1);
        outake = new Outake(commandLoop, transfer);
        shoot = new Shoot(commandLoop, transfer, shooter, follower, telemetryM);
        turretHoodUpdate = new TurretHoodUpdate(commandLoop, turret, hoodServo, follower);
    }


}
