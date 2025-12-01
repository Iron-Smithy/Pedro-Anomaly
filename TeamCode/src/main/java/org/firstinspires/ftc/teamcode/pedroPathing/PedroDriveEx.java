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

import java.util.function.Supplier;

@Configurable
@TeleOp
public class PedroDriveEx extends OpMode {

    // --- Hardware & Follower ---
    private Follower follower;
    public static Pose startingPose; // Set from autonomous if needed

    private MotorGroup outtakeMotors;

    // --- Telemetry ---
    private TelemetryManager telemetryM;

    // --- Drive State ---
    private boolean automatedDrive = false;
    private boolean isRobotCentric = false;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.25;
    private final double k = 2.0;
    private final double expKMinus1 = Math.exp(k) - 1;

    // --- Intake / Outtake ---
    private int intakeDir = 0;       // -1 intake reverse, 1 intake forward
    private int outtakeDir = 1;      // 1 outtake forward
    private long outtakeTPS = 2000;
    public static double indexingWheelDir = 0;

    // --- Button Handlers ---
    private ButtonHandler indexingControl = new ButtonHandler();
    private ButtonHandler outtakeControl = new ButtonHandler();
    private ButtonHandler intakeDirectionControlU = new ButtonHandler();
    private ButtonHandler intakeDirectionControlD = new ButtonHandler();
    private ButtonHandler outtakeVelocityU = new ButtonHandler();
    private ButtonHandler outtakeVelocityD = new ButtonHandler();
    private ButtonHandler slowTrigger = new ButtonHandler();
    private ButtonHandler robotCentricControl = new ButtonHandler();
    private ButtonHandler rotationReset = new ButtonHandler();

    // --- Path Following ---
    private Supplier<PathChain> pathChain;

    @Override
    public void init() {
        // Initialize hardware first
        RobotHardware.init(hardwareMap);

        // Initialize motor groups AFTER hardware is initialized
        outtakeMotors = new MotorGroup(RobotHardware.outtakeMotor1, RobotHardware.outtakeMotor2);
        outtakeMotors.setUsingEncoder();

        // Setup follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        // Telemetry
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // --- Button Actions ---

        // Intake buttons
        intakeDirectionControlD.setOnPress(() -> intakeDir = (intakeDir == 1 ? 0 : 1));
        intakeDirectionControlU.setOnPress(() -> intakeDir = (intakeDir == -1 ? 0 : -1));

        // Outtake buttons
        indexingControl.setOnHold(() -> indexingWheelDir = 1);
        indexingControl.setOnRelease(() -> indexingWheelDir = 0);
        outtakeControl.setOnPress(() -> outtakeDir = 1 - outtakeDir);

        // Outtake power adjustments
        outtakeVelocityU.setOnPress(() -> outtakeTPS = Math.min(3250, outtakeTPS + 250));
        outtakeVelocityD.setOnPress(() -> outtakeTPS = Math.max(500, outtakeTPS - 250));

        // Robot drive
        slowTrigger.setOnPress(() -> slowMode = !slowMode);
        robotCentricControl.setOnPress(() -> isRobotCentric = !isRobotCentric);

        // FieldCentric rotation reset
        rotationReset.setOnPress(() -> follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), Math.toRadians(0))));

        // --- Lazy path builder ---
        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
    }

    @Override
    public void start() {
        follower.startTeleopDrive(true); // true enables brake mode
    }

    @Override
    public void loop() {
        // --- Update Follower and Telemetry ---
        follower.update();
        telemetryM.update();

        // --- Update button handlers ---
        outtakeControl.update(gamepad1.left_trigger, 0.25f);
        intakeDirectionControlU.update(gamepad1.right_bumper);
        indexingControl.update(gamepad1.right_trigger, 0.25f);
        intakeDirectionControlD.update(gamepad1.left_bumper);
        outtakeVelocityU.update(gamepad1.dpad_up);
        outtakeVelocityD.update(gamepad1.dpad_down);
        slowTrigger.update(gamepad1.left_stick_button);
        robotCentricControl.update(gamepad1.share);
        rotationReset.update(gamepad1.options);


        // --- Drive ---
        if (!automatedDrive) {
            if (!slowMode) {
                // Exponential scaling for more precise control
                follower.setTeleOpDrive(
                        scaleStick(-gamepad1.left_stick_y),
                        scaleStick(-gamepad1.left_stick_x),
                        -gamepad1.right_stick_x / 2.0,
                        isRobotCentric
                );
            } else {
                // Slow mode
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y * slowModeMultiplier,
                        -gamepad1.left_stick_x * slowModeMultiplier,
                        -gamepad1.right_stick_x * slowModeMultiplier,
                        isRobotCentric
                );
            }
        }

        // --- Intake / Outtake ---
        RobotHardware.intakeMotor.setPower(intakeDir);
        outtakeMotors.setVelocity(outtakeTPS * outtakeDir);
        RobotHardware.indexingWheel.setPower(gamepad1.a ? -indexingWheelDir : indexingWheelDir);

        // --- Debug Telemetry ---
        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);

        telemetry.addData("slowModeMultiplier", slowModeMultiplier);
        telemetry.addData("slowModeActive", slowMode);
        telemetry.addData("Outtake Power", outtakeTPS);
        telemetry.addData("Outtake Dir", outtakeDir);
        telemetry.addData("Motor Speed", outtakeTPS * outtakeDir);
        telemetry.update();
    }

    private double scaleStick(double value) {
        return Math.signum(value) * (Math.exp(k * Math.abs(value)) - 1) / expKMinus1;
    }
}
