//package pedroPathing;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.pedropathing.control.FilteredPIDFCoefficients;
//import com.pedropathing.control.PIDFCoefficients;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.follower.FollowerConstants;
//import com.pedropathing.ftc.FollowerBuilder;
//import com.pedropathing.ftc.drivetrains.MecanumConstants;
//import com.pedropathing.ftc.localization.Encoder;
//import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
//import com.pedropathing.ftc.localization.constants.PinpointConstants;
//import com.pedropathing.paths.PathConstraints;
//import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//
//@Config
//public class Constants {
//
//    public static FollowerConstants followerConstants = new FollowerConstants()
//            .mass(10)
//            .forwardZeroPowerAcceleration(-33)
//            .lateralZeroPowerAcceleration(-63)
//            .useSecondaryTranslationalPIDF(false)
//            .useSecondaryHeadingPIDF(true)
//            .useSecondaryDrivePIDF(true)
//            .centripetalScaling(0.0006)
//            .automaticHoldEnd(true)
//            .translationalPIDFCoefficients(new PIDFCoefficients(0.08, 0, 0.0005, 0))
//            .headingPIDFCoefficients(new PIDFCoefficients(0.5, 0, 0.01, 0))
//            .drivePIDFCoefficients(new FilteredPIDFCoefficients(1, 0, 0.01, 0.6, 0.01))
//            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2, 0, 0.01, 0.1))
//            .secondaryDrivePIDFCoefficients(
//                    new FilteredPIDFCoefficients(0.012, 0, 0.0001, 0, 0.01)
//            );
//
//    public static MecanumConstants driveConstants = new MecanumConstants()
//            .leftFrontMotorName("leftFront")
//            .leftRearMotorName("leftRear")
//            .rightFrontMotorName("rightFront")
//            .rightRearMotorName("rightRear")
//            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
//            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
//            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
//            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
//            .xVelocity(80)
//            .yVelocity(64)
//            .useVoltageCompensation(true);
//
//    public static PinpointConstants localizerConstants = new PinpointConstants()
//            .forwardPodY(1.57)
//            .strafePodX(-4.72)
//            .distanceUnit(DistanceUnit.INCH)
//            .hardwareMapName("pinpoint")
//            .encoderResolution(
//                    GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
//            )
//            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
//            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
//
//    public static PathConstraints pathConstraints = new PathConstraints(
//            0.99,
//            100,
//            1,
//            1
//    );
//
//    public static Follower createFollower(HardwareMap hardwareMap) {
//        return new FollowerBuilder(followerConstants, hardwareMap)
//                .mecanumDrivetrain(driveConstants)
//                .pinpointLocalizer(localizerConstants)
//                .pathConstraints(pathConstraints)
//                .build();
//    }
//}