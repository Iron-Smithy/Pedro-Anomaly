package org.firstinspires.ftc.teamcode.pedroPathing.Teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.tools.Actions.*;
import org.firstinspires.ftc.teamcode.tools.ButtonHandler;

@Configurable
@TeleOp
public class PDriveARREc extends OpMode {

    /* DRIVE SYSTEM */
    private Follower follower;
    public static Pose startingPose = new Pose(120, 127, Math.toRadians(37));
    private boolean automatedDrive = false;

    // Field-centric
    private double fieldCentricOffset = 0;

    // Drive shaping
    private final double k = 2.0;
    private final double expKMinus1 = Math.exp(k) - 1;
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

    /* AUTOSCORE TASKS */
    private AutoScoreTaskE scorePosCircle;
    private AutoScoreTaskE scorePosSquare;
    private AutoScoreTaskE scorePosTriangle;
    private AutoScoreTaskE scorePosCross;
    private AutoScoreTaskE activeScoreTask = null;

    @Override
    public void init() {
        RobotHardware.init(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose != null ? startingPose : new Pose());
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        intake = new IntakeAction(hardwareMap);
        indexer = new IndexAction(hardwareMap);
        ejector = new EjectorAction(hardwareMap);
        outtake = new OuttakeAction(hardwareMap);

        // Outtake toggles
        outtakeControl.setOnPress(() -> outtakeON = !outtakeON);
        outtakeVelocityUp.setOnPress(() -> outtakeSpeed = Math.min(outtakeSpeedMax, outtakeSpeed + 150));
        outtakeVelocityDown.setOnPress(() -> outtakeSpeed = Math.max(outtakeSpeedMin, outtakeSpeed - 150));

        intakeDirDown.setOnPress(() -> intakePow = (intakePow > 0 ? 0 : 1));
        intakeDirUp.setOnPress(() -> intakePow = (intakePow < 0 ? 0 : -1));

        // Create scoring tasks with target pose & TPS
        scorePosCircle = new AutoScoreTaskE(follower, outtake, indexer, ejector, new Pose(84, 84, Math.toRadians(47)), 1000);
        scorePosSquare = new AutoScoreTaskE(follower, outtake, indexer, ejector, new Pose(108, 108, Math.toRadians(47)), 1000);
        scorePosTriangle = new AutoScoreTaskE(follower, outtake, indexer, ejector, new Pose(84, 12, Math.toRadians(65)), 1000);
        scorePosCross = new AutoScoreTaskE(follower, outtake, indexer, ejector, new Pose(84, 132, Math.toRadians(7)), 1000);
    }

    @Override
    public void start() {
        follower.startTeleopDrive(true);
        outtake.spinUp(outtakeSpeed);
        outtake.stop();
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();

        // outtakeControl.update(gamepad1.left_trigger, 0.25f);
        outtakeVelocityUp.update(gamepad1.dpad_up);
        outtakeVelocityDown.update(gamepad1.dpad_down);
        intakeDirUp.update(gamepad1.right_bumper);
        // intakeDirDown.update(gamepad1.circle); // ============ rebind===============================

        slowMode = gamepad1.left_stick_button;

        /*--------------------*
         * MANUAL DRIVE       *
         *--------------------*/
        if (activeScoreTask == null && !automatedDrive) {
            double heading = follower.getHeading() - fieldCentricOffset;
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x / 2;

            double driveY = Math.signum(y) * (Math.exp(k * Math.abs(y)) - 1) / expKMinus1;
            double driveX = Math.signum(x) * (Math.exp(k * Math.abs(x)) - 1) / expKMinus1;

            if (slowMode) {
                driveX *= slowModeMultiplier;
                driveY *= slowModeMultiplier;
                turn   *= slowModeMultiplier;
            }

            double rotX = driveX * Math.cos(-heading) - driveY * Math.sin(-heading);
            double rotY = driveX * Math.sin(-heading) + driveY * Math.cos(-heading);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
            double frontLeftPower = (rotY + rotX + turn) / denominator;
            double backLeftPower = (rotY - rotX + turn) / denominator;
            double frontRightPower = (rotY - rotX - turn) / denominator;
            double backRightPower = (rotY + rotX - turn) / denominator;

            RobotHardware.frontLeftMotor.setPower(frontLeftPower);
            RobotHardware.backLeftMotor.setPower(backLeftPower);
            RobotHardware.frontRightMotor.setPower(frontRightPower);
            RobotHardware.backRightMotor.setPower(backRightPower);
        }

        /*--------------------*
         * AUTO SCORE BUTTONS *
         *--------------------*/
        if (gamepad1.circleWasPressed()) startAutoScoreTask(scorePosCircle);
        if (gamepad1.squareWasPressed()) startAutoScoreTask(scorePosSquare);
        if (gamepad1.triangleWasPressed()) startAutoScoreTask(scorePosTriangle);
        if (gamepad1.crossWasPressed()) startAutoScoreTask(scorePosCross);

        // Manual override
        if (gamepad1.optionsWasPressed() && activeScoreTask != null) {
            activeScoreTask.cancel();
            activeScoreTask = null;
        }

        // Update active task
        if (activeScoreTask != null) {
            activeScoreTask.update(outtakeSpeed); // pass current shooter speed
        }

        /*--------------------*
         * SUBSYSTEMS         *
         *--------------------*/
        if (outtakeON) {
            outtake.spinUp(outtakeSpeed);
            double rumble = outtake.isAtTargetVelocity() ? 0.8 : 0.1;
            gamepad1.rumble(rumble, rumble, 50);
        } else {
            outtake.stop();
            gamepad1.rumble(0, 0, 0);
        }

        if (Math.abs(intakePow) > 0) intake.spin(intakePow);
        else intake.stop();

        if (gamepad1.left_bumper) indexer.runIn();
        else indexer.stop();

        if (gamepad1.a) ejector.up();
        else ejector.down();

        if (gamepad1.shareWasPressed()) fieldCentricOffset = follower.getHeading();

        /*--------------------*
         * TELEMETRY          *
         *--------------------*/
        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
        telemetry.addData("Slow Mode %", slowModeMultiplier);
        telemetry.addData("Slow Mode Active", slowMode);
        telemetry.addData("Outtake Active", outtakeON);
        telemetry.addData("Outtake Speed", outtakeSpeed);
        telemetry.update();
    }

    private void startAutoScoreTask(AutoScoreTaskE task) {
        if (activeScoreTask != null) activeScoreTask.cancel();
        outtakeSpeed = task.startTPS;
        outtakeON = true;
        activeScoreTask = task;
        activeScoreTask.start();
    }
}
