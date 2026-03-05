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
            .mass(14.2)
            .forwardZeroPowerAcceleration(-35.65)
            .lateralZeroPowerAcceleration(-70.41)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0.00001, 0.1, 0.02))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.02,0.0000,0.01,0.01))
            .headingPIDFCoefficients(new PIDFCoefficients(0.7, 0.000, 0.15, 0.03))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.07,0.000,0.01,0.02))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(1.2,0.000,0.006,0,0.02))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.035,0.0000,0.001,0,0.002))
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .centripetalScaling(0.005);


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.9, 0.25);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(92.5)
            .strafePodX(141)
            .distanceUnit(DistanceUnit.MM)
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
            .xVelocity(73.77)
            .yVelocity(52);
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }

}