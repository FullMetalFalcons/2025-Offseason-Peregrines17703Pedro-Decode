package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Configurable
public class Constants {

    // XAVIER NOTE:  While tuning for Autonomous, the Pedro Pathing documentation will tell you to copy and paste over a "MecanumConstants" declaration statement.
    //   We already did that below, so just add on to it or modify it as necessary. You will still need to copy and paste certain things over, though,
    //   such as a localizerConstants declaration statement

    final static double lbPerKg = 2.205;
    final static double robotWeightInPounds = 20; // TODO: Update with your robot's actual weight
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(robotWeightInPounds / lbPerKg)
            .headingPIDFCoefficients(new PIDFCoefficients(0.65,0,0.1,0))
            .translationalPIDFCoefficients(new PIDFCoefficients(0, 0, 0, 0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0, 0, 0, 0, 0))
            .forwardZeroPowerAcceleration(0)
            .lateralZeroPowerAcceleration(0)
            //i like boys
            //.predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0, 0, 0)) // (kP, kLinear, kQuadratic)
            .centripetalScaling(0)
            ;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("front_right") // TODO: Set these names to the drive motor names used in your configuration file
            .rightRearMotorName("back_right")
            .leftRearMotorName("back_left")
            .leftFrontMotorName("front_left")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE) // TODO: Set these directions based on your robot's drive motors
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(82.64552415262057)
            .yVelocity(66.42505171167569)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            1,
            1);
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-2.444)
            .strafePodX(-2.278244)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}