package org.firstinspires.ftc.teamcode.states;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class Turret {
    //TODO: make sure panels stuff is done correctly
    TelemetryManager telemetryM;
    ElapsedTime turretTime = new ElapsedTime();
    double lastTurretError;
    double turretIntegral;
    public static double turretKp = 0;
    public static double turretKd = 0;
    public static double turretKi = 0;
    public void turnTurret(double position){
        double error = position-(0); //TODO: change 0 to getPosition
        double dt = turretTime.seconds();
        if (dt < 0.0001) dt = 0.0001;
        turretIntegral += error* dt;
        double derivative = (error- lastTurretError)/ dt;
        lastTurretError = error;

        turretTime.reset();

        double output;
        output = (error * turretKp) + (derivative * turretKd) + (turretIntegral * turretKi) ;

        //TODO: need to change these to whatever we name the motor
        //robot.flyRight.setPower(output);
        //robot.flyLeft.setPower(output);
        telemetryM.addData("turret motor power", Math.max(-1, Math.min(1, output)));
    }

}
