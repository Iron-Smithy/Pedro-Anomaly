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
    private boolean firstRun = true;

    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    private boolean intakeToggle = false;
    private int intakeDir = -1;
    private int outtakeDir = 1; // 1 outtake
    private double outtakePow = 0.5;
    public static double indexingWheelDir = 0; // 1 intake
    MotorGroup outtakeMotors = new MotorGroup(outtakeMotor1, outtakeMotor2);
    ButtonHandler leftPaddle = new ButtonHandler();
    ButtonHandler rightPaddle = new ButtonHandler();
    ButtonHandler leftTrigger = new ButtonHandler();
    ButtonHandler rightTrigger = new ButtonHandler();
    ButtonHandler DPadUP = new ButtonHandler();
    ButtonHandler DPadDown = new ButtonHandler();
    double k = 2.0;
    double expKMinus1 = Math.exp(k) - 1; // Precompute at init;

    @Override
    public void init() {
        RobotHardware.init(hardwareMap);

        outtakeMotors.setUsingEncoder();

        // Intake
        rightPaddle.setOnPress(() -> intakeDir = (intakeDir == 1 ? 0 : 1));
        rightTrigger.setOnPress(() -> intakeDir = (intakeDir == -1 ? 0 : -1));

        // Outtake
        leftPaddle.setOnHold(() -> indexingWheelDir = 1);
        leftPaddle.setOnRelease(() -> indexingWheelDir = 0);
        leftTrigger.setOnPress(() -> outtakeDir = 1 - outtakeDir);

        DPadUP.setOnPress(() -> outtakePow = Math.min(1.0, Math.max(0, outtakePow + 0.05)));
        DPadDown.setOnPress(() -> outtakePow = Math.min(1.0, Math.max(0, outtakePow - 0.05)));

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
        if (firstRun) {
            firstRun = false;
            return;
        }

        //Call this once per loop
        follower.update();
        telemetryM.update();

        leftPaddle.update(gamepad1.left_bumper);
        rightPaddle.update(gamepad1.right_bumper);

        leftTrigger.update(gamepad1.left_trigger, 0.25f);
        rightTrigger.update(gamepad1.right_trigger, 0.25f);

        DPadUP.update(gamepad1.dpad_up);
        DPadDown.update(gamepad1.dpad_down);

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

        //Slow Mode
        if (gamepad1.dpad_left) {
            slowMode = !slowMode;
        }

        //Optional way to change slow mode strength
        if (gamepad1.xWasPressed()) {
            slowModeMultiplier += 0.25;
        }

        //Optional way to change slow mode strength
        if (gamepad2.bWasPressed()) {
            slowModeMultiplier -= 0.25;
        }

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