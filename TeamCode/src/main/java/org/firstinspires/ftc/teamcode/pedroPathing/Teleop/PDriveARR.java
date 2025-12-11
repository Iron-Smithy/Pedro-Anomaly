package org.firstinspires.ftc.teamcode.pedroPathing.Teleop;

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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.MConstants;
import org.firstinspires.ftc.teamcode.tools.Actions.*;
import org.firstinspires.ftc.teamcode.tools.ButtonHandler;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class PDriveARR extends OpMode {

    /* DRIVE SYSTEM */
    private Follower follower;
    public static Pose startingPose = new Pose(120, 127, Math.toRadians(37));
    private boolean automatedDrive = false;
    private Supplier<PathChain> pathChain;

    // Field-centric alignment
    private double fieldCentricOffset = 0;

    // Drive shaping
    private final double k = 2.0;  // exponential factor
    private final double expKMinus1 = Math.exp(k) - 1;

    // Slow mode
    private boolean slowMode = false;
    private final double slowModeMultiplier = 0.25;


    /* TELEMETRY */
    private TelemetryManager telemetryM;


    /* BUTTON CONTROL */
    private final ButtonHandler outtakeControl = new ButtonHandler();
    private final ButtonHandler outtakeVelocityUp = new ButtonHandler();
    private final ButtonHandler outtakeVelocityDown = new ButtonHandler();

    private final ButtonHandler intakeDirUp = new ButtonHandler();
    private final ButtonHandler intakeDirDown = new ButtonHandler();


    /* SUBSYSTEMS */
    private IntakeAction intake;
    private IndexAction indexer;
    private EjectorAction ejector;
    private OuttakeAction outtake;

    private boolean outtakeON = false;
    private long outtakeSpeed = 1000;
    private final long outtakeSpeedMin = 500;
    private final long outtakeSpeedMax = 3000;

    private double intakePow = 0;


    /* INIT */
    @Override
    public void init() {
        RobotHardware.init(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose != null ? startingPose : new Pose());
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Lazy path generation
        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(
                        follower::getPose,
                        new Pose(100, 107)
                )))
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(
                                follower::getHeading,
                                Math.toRadians(47),
                                0.8
                        )
                )
                .build();

        // Subsystem init
        intake = new IntakeAction(hardwareMap);
        indexer = new IndexAction(hardwareMap);
        ejector = new EjectorAction(hardwareMap);
        outtake = new OuttakeAction(hardwareMap);

        // Outtake control setup
        outtakeControl.setOnPress(() ->
                outtakeON = !outtakeON
        );

        outtakeVelocityUp.setOnPress(() ->
                outtakeSpeed = Math.min(outtakeSpeedMax, outtakeSpeed + 150)
        );

        outtakeVelocityDown.setOnPress(() ->
                outtakeSpeed = Math.max(outtakeSpeedMin, outtakeSpeed - 150)
        );

        // Intake direction controls
        intakeDirDown.setOnPress(() -> intakePow = (intakePow > 0 ? 0 : 1));
        intakeDirUp.setOnPress(() -> intakePow = (intakePow < 0 ? 0 : -1));
    }


    /* START */
    @Override
    public void start() {
        follower.startTeleopDrive(true);

        // Pre-spin outtake to avoid startup delay
        outtake.spinUp(outtakeSpeed);
        outtake.stop();
    }


    /* LOOP */
    @Override
    public void loop() {
        follower.update();
        telemetryM.update();

        // follower.setPose(aprilPose);

        // Button updates
        outtakeControl.update(gamepad1.left_trigger, 0.25f);
        outtakeVelocityUp.update(gamepad1.dpad_up);
        outtakeVelocityDown.update(gamepad1.dpad_down);

        intakeDirUp.update(gamepad1.right_bumper);
        intakeDirDown.update(gamepad1.circle);


        /*------------------------------------*
         *       MANUAL DRIVING MODE          *
         *------------------------------------*/
        if (!automatedDrive) {
            double heading = follower.getHeading() - fieldCentricOffset;

            // Joystick input
            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x / 2;

            // Exponential shaping
            double driveY = Math.signum(y)  * (Math.exp(k * Math.abs(y)) - 1) / expKMinus1;
            double driveX = Math.signum(x)  * (Math.exp(k * Math.abs(x)) - 1) / expKMinus1;

            // Slow mode
            if (slowMode) {
                driveX *= slowModeMultiplier;
                driveY *= slowModeMultiplier;
                turn   *= slowModeMultiplier;
            }

//            follower.driveFieldCentric(driveX, driveY, turn, heading);
        }


        /*------------------------------------*
         *    AUTOMATED PATH FOLLOWING        *
         *------------------------------------*/
        if (gamepad1.shareWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        if (automatedDrive && (gamepad1.psWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive(true);
            automatedDrive = false;
        }

        // Reset field-centric alignment
        if (gamepad1.optionsWasPressed()) {
            fieldCentricOffset = follower.getHeading();
        }

        slowMode = gamepad1.left_stick_button;

        /*------------------------------------*
         *           SUBSYSTEMS               *
         *------------------------------------*/

        // Outtake
        if (outtakeON) {
            outtake.spinUp(outtakeSpeed);

            double rumbleStrength = outtake.isAtTargetVelocity() ? 0.8 : 0.1;
            gamepad1.rumble(rumbleStrength, rumbleStrength, 50);
        } else {
            outtake.stop();
            gamepad1.rumble(0, 0, 0);
        }

        // Intake
        if (Math.abs(intakePow) > 0) intake.spin(intakePow);
        else intake.stop();

        // Indexer
        if (gamepad1.left_bumper) indexer.runIn();
        else indexer.stop();

        // Ejector
        if (gamepad1.a) ejector.up();
        else ejector.down();


        /*------------------------------------*
         *           TELEMETRY                *
         *------------------------------------*/
        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);

        telemetry.addData("Slow Mode %", slowModeMultiplier);
        telemetry.addData("Slow Mode Active", slowMode);
        telemetry.addData("Outtake Active", outtakeON);
        telemetry.addData("Outtake Speed", outtakeSpeed);
        telemetry.update();
    }
}
