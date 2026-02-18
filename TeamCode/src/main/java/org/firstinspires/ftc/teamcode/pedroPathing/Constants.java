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
            .mass(7.172429351)
            .forwardZeroPowerAcceleration(-38.33066785395804)
            .lateralZeroPowerAcceleration(-74.597216695261)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.4, 0, 0.05, 0.045))
            .headingPIDFCoefficients(new PIDFCoefficients(0.9, 0, 0.1, 0.05))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.045, 0, 0.00002, 0.25, 0.02));
//            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.06, 0, 0.00001, 0.25, 0.02));

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRightMotor")
            .rightRearMotorName("backRightMotor")
            .leftRearMotorName("backLeftMotor")
            .leftFrontMotorName("frontLeftMotor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(66.516300028932956666666666666667)
            .yVelocity(49.182133995980976);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(1.0/4)
            .strafePodX(1.0/4)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

//    public static PathConstraints pathConstraints = new PathConstraints(0.8, 120, 0.8, 1);
    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            0.1,
            0.1,
            0.009,
            50,
            1.3,
            10,
            5
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}