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

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(14.3)
            .forwardZeroPowerAcceleration(-47.06)
            .lateralZeroPowerAcceleration(-77.61)
//            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0.00001, 0.1, 0.02))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.4,0.0000,0.04,0.02))
//            .headingPIDFCoefficients(new PIDFCoefficients(0.7, 0.000, 0.15, 0.03))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.6,0.000,0.15,0.03))
//            .drivePIDFCoefficients(new FilteredPIDFCoefficients(1.2,0.000,0.006,0,0.02))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.035,0.0000,0.001,0,0.002))
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(1, 0, 0.04, 0.06))
            .headingPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0.08, 0.07))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.6, 0, 0.04, 0, 0.03))
            .centripetalScaling(0.005);


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.9, 0.25);

    public static PinpointConstants localizerConstants = new PinpointConstants()
//            .forwardPodY(92.5)
//            .strafePodX(141)
//            .distanceUnit(DistanceUnit.MM)
            .strafePodX(-3.5433070866)
            .forwardPodY(4.2519685039)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(72.27)
            .yVelocity(44.7);
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }

}