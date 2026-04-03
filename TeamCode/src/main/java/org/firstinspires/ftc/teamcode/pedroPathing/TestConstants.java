package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Deprecated
public class TestConstants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.62)

            //TODO: delete this todo once we input forward zero power
            .forwardZeroPowerAcceleration(-34.545892)

            //TODO: this runs in the direction it says it will on the DS
            .lateralZeroPowerAcceleration(-53.881418)

            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.00))
            .headingPIDFCoefficients(new PIDFCoefficients(0.68, 0, 0.02, 0.0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.43, 0.0, 0.01, 0.6, 0.00))
            //TODO: centripetal probably wont change
            .centripetalScaling(0.0005);
//    public static MecanumConstants driveConstants = new MecanumConstants()
//            .maxPower(1)
//            .rightFrontMotorName("fr")
//            .rightRearMotorName("br")
//            .leftRearMotorName("bl")
//            .leftFrontMotorName("fl")
//            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
//            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
//            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
//            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
//            //.useBrakeModeInTeleOp(true)
//            .xVelocity(72.6066627)
//            .yVelocity(63.8361884);
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .forwardPodY(-1)
            .strafePodX(-2.5);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 0.1, 0.1, 0.01, 50, 2, 10, 1.5);
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
//                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
