package org.firstinspires.ftc.teamcode.pedroPathing;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.tools.ButtonHandler;
import org.firstinspires.ftc.teamcode.tools.MotorGroup;
import static org.firstinspires.ftc.teamcode.RobotHardware.*;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class PedroDrive extends OpMode {
    private MotorGroup outtakeMotors;

    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.25;

    private boolean intakeToggle = false;

    private boolean isRobotCentric = false;
    private int intakeDir = -1;
    private int outtakeDir = 1; // 1 outtake
    private double outtakePow = 0.5;
    public static double indexingWheelDir = 0; // 1 intake

    private ButtonHandler indexingControl = new ButtonHandler();
    private ButtonHandler outtakeControl = new ButtonHandler();
    private ButtonHandler intakeDirectionControlU = new ButtonHandler();
    private ButtonHandler intakeDirectionControlD = new ButtonHandler();
    private ButtonHandler outtakeVelocityU = new ButtonHandler();
    private ButtonHandler outtakeVelocityD = new ButtonHandler();
    private ButtonHandler slowTrigger = new ButtonHandler();
    private ButtonHandler robotCentricControl = new ButtonHandler();
    private ButtonHandler rotationReset = new ButtonHandler();
    double k = 2.0;
    double expKMinus1 = Math.exp(k) - 1; // Precompute at init;

    @Override
    public void init() {
        RobotHardware.init(hardwareMap);

        outtakeMotors = new MotorGroup(RobotHardware.outtakeMotor1, RobotHardware.outtakeMotor2);
        outtakeMotors.setUsingEncoder();

        // Intake
        intakeDirectionControlD.setOnPress(() -> intakeDir = (intakeDir == 1 ? 0 : 1));
        intakeDirectionControlU.setOnPress(() -> intakeDir = (intakeDir == -1 ? 0 : -1));

        // Outtake
        indexingControl.setOnHold(() -> indexingWheelDir = 1);
        indexingControl.setOnRelease(() -> indexingWheelDir = 0);
        outtakeControl.setOnPress(() -> outtakeDir = 1 - outtakeDir);

        outtakeVelocityU.setOnPress(() -> outtakePow = Math.min(1.0, Math.max(0, outtakePow + 0.05)));
        outtakeVelocityD.setOnPress(() -> outtakePow = Math.min(1.0, Math.max(0, outtakePow - 0.05)));

        slowTrigger.setOnPress(() -> slowMode = !slowMode);
        robotCentricControl.setOnPress(() -> isRobotCentric = !isRobotCentric);

        // FieldCentric rotation reset
        rotationReset.setOnPress(() -> follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), Math.toRadians(0))));

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();

        outtakeControl.update(gamepad1.left_trigger, 0.25f);
        intakeDirectionControlU.update(gamepad1.right_bumper);
        indexingControl.update(gamepad1.right_trigger, 0.25f);
        intakeDirectionControlD.update(gamepad1.left_bumper);
        outtakeVelocityU.update(gamepad1.dpad_up);
        outtakeVelocityD.update(gamepad1.dpad_down);
        slowTrigger.update(gamepad1.left_stick_button);
        robotCentricControl.update(gamepad1.share);
        rotationReset.update(gamepad1.options);

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    Math.signum(-gamepad1.left_stick_y) * (Math.exp(k * Math.abs(-gamepad1.left_stick_y)) - 1) / expKMinus1,
                    Math.signum(-gamepad1.left_stick_x) * (Math.exp(k * Math.abs(-gamepad1.left_stick_x)) - 1) / expKMinus1,
                    -gamepad1.right_stick_x / 2,
                    false // Robot Centric
            );

                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    false // Robot Centric
            );
        }

        //Automated PathFollowing
//        if (gamepad1.aWasPressed()) {
//            follower.followPath(pathChain.get());
//            automatedDrive = true;
//        }

        //Stop automated following if the follower is done
//        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
//            follower.startTeleopDrive();
//            automatedDrive = false;
//        }

        intakeMotor.setPower(intakeDir);
        outtakeMotors.setPower(outtakePow * outtakeDir);
        indexingWheel.setPower(gamepad1.a ? indexingWheelDir*-1 : indexingWheelDir);

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);

        telemetry.addData("slowModeMultiplier", slowModeMultiplier);
        telemetry.addData("slow on?", slowMode);
        telemetry.addData("Outtake Speed", outtakePow);
        telemetry.addData("Outtake Dir", outtakeDir);
        telemetry.addData("Motor Speed", outtakePow * outtakeDir);
        telemetry.update();
    }
}