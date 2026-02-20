//package org.firstinspires.ftc.teamcode.RIstates;
//
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//import com.pedropathing.follower.Follower;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Hood;
//import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Intake;
//import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Limelight.Limelight;
//import org.firstinspires.ftc.teamcode.RIstates.management.Systems.TakaShooter;
//import org.firstinspires.ftc.teamcode.RIstates.management.Systems.Turret;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//public class TeleOp4 extends OpMode {
//    Turret turret;
//    Hood hood;
//    Limelight limelight;
//    Intake intake;
//    TakaShooter shooter;
//    Follower follower;
//    TelemetryManager t;
//    @Override
//    public void init() {
//        follower = Constants.createFollower(hardwareMap);
//        t = PanelsTelemetry.INSTANCE.getTelemetry();
//        turret = new Turret(hardwareMap, follower, "t");
//        hood = new Hood(hardwareMap, "h");
//        intake = new Intake(hardwareMap, "i");
//        shooter = new TakaShooter(hardwareMap, "rs", "ls");
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//        t.update();
//
//        turret.trackGoal(1);
//        shooter.shoot();
//    }
//}
