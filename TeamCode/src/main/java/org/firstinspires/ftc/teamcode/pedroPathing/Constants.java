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
            .forwardZeroPowerAcceleration(-29.626717953837375395)
//              -33.2969465834127 + -26.271693488531678 + -29.08701012318395637 + -31.21637516957636 +  -31.68073159228111 + -26.207550766038448 / 6
            .lateralZeroPowerAcceleration(-48.051936107175094)
//              -52.285478414558014 + -48.67520642692837 + -40.94391386741864 + -48.42298680485524 + -43.11498647191354 + -54.86904465737676 / 6
            .translationalPIDFCoefficients(new PIDFCoefficients(0.3, 0, 0.05, 0.01))
            .headingPIDFCoefficients(new PIDFCoefficients(0.9, 0, 0.1, 0.06))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.02, 0, 0.000005, 0.2, 0.01));
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
            .xVelocity(80.991705306245903333333333333333)
//              81.19251113050566 + 78.192756712906 + 83.58984807532605 / 3
            .yVelocity(66.516300028932956666666666666667);
//              66.500956823696 + 67.42362243171753 + 65.62432083138534 / 3

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(0)
            .strafePodX(0)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.5, 100, 0.3, 0.6); // default 0.99, 100, 1, 1

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
