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
import org.firstinspires.ftc.teamcode.tools.Actions.AutoDriveTask;
import org.firstinspires.ftc.teamcode.tools.Actions.AutoFireTask;
import org.firstinspires.ftc.teamcode.tools.Actions.EjectorAction;
import org.firstinspires.ftc.teamcode.tools.Actions.IndexAction;
import org.firstinspires.ftc.teamcode.tools.Actions.IntakeAction;
import org.firstinspires.ftc.teamcode.tools.Actions.OuttakeAction;
import org.firstinspires.ftc.teamcode.tools.ButtonHandler;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class PDriveA_RED_sF extends OpMode {

    /* DRIVE SYSTEM */
    private Follower follower;
    public static Pose startingPose = new Pose(/*144-*/125, 87, Math.toRadians(/*180-*/0)); //new Pose(120, 127, Math.toRadians(37));

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

    /* AUTO TASKS */
    private AutoDriveTask driveCircle;
    private AutoDriveTask driveSquare;
    private AutoDriveTask driveTriangle;
    private AutoDriveTask driveCross;
    private AutoDriveTask drivePark;
    private AutoFireTask fireTask = null;

    private Supplier<PathChain> park;

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

        outtakeVelocityUp.setOnPress(() -> outtakeSpeed = Math.min(outtakeSpeedMax, outtakeSpeed + 150));
        outtakeVelocityDown.setOnPress(() -> outtakeSpeed = Math.max(outtakeSpeedMin, outtakeSpeed - 150));
        intakeDirDown.setOnPress(() -> intakePow = (intakePow > 0 ? 0 : MConstants.intakePowerOut));
        intakeDirUp.setOnPress(() -> intakePow = (intakePow < 0 ? 0 : MConstants.intakePowerIn));

        fieldCentricOffset = follower.getHeading();

        // Initialize auto-drive tasks
        driveCircle   = new AutoDriveTask(follower, new Pose(84, 84, Math.toRadians(47)));
        driveSquare   = new AutoDriveTask(follower, new Pose(108, 108, Math.toRadians(47)));
        driveTriangle = new AutoDriveTask(follower, new Pose(84, 20, Math.toRadians(65)));
        driveCross    = new AutoDriveTask(follower, new Pose(84, 132, Math.toRadians(7)));
        drivePark     = new AutoDriveTask(follower, new Pose (38.6, 40, Math.toRadians(90)));

    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        outtake.spinUp(outtakeSpeed);
        ejector.down();
//        outtake.stop();
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();

        // ----------------------------
        // MANUAL DRIVE
        // ----------------------------
        if (!anyDriveTaskActive()) {
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
            RobotHardware.frontLeftMotor.setPower((rotY + rotX + turn) / denominator);
            RobotHardware.backLeftMotor.setPower((rotY - rotX + turn) / denominator);
            RobotHardware.frontRightMotor.setPower((rotY - rotX - turn) / denominator);
            RobotHardware.backRightMotor.setPower((rotY + rotX - turn) / denominator);

            // ----------------------------
            // BUTTON UPDATES
            // ----------------------------
            outtakeVelocityUp.update(gamepad1.dpad_up);
            outtakeVelocityDown.update(gamepad1.dpad_down);
            intakeDirUp.update(gamepad1.left_bumper);
            intakeDirDown.update(gamepad1.left_trigger, 0.25F);

            slowMode = gamepad1.left_stick_button;

            // ----------------------------
            // SUBSYSTEMS
            // ----------------------------

            if (intakePow == MConstants.intakePowerIn) intake.runIn();
            else if (intakePow == MConstants.intakePowerOut) intake.runOut();
            else intake.stop();

            if (gamepad1.psWasPressed()) {
                follower.setPose(new Pose(120, 127, Math.toRadians(37)));
                fieldCentricOffset = Math.toRadians(0);
            }
        }

        if (gamepad1.shareWasPressed()) fieldCentricOffset = follower.getHeading();

        // ----------------------------
        // AUTO-DRIVE BUTTONS
        // ----------------------------
        if (gamepad1.circleWasPressed())   { outtakeSpeed = 1200; cancelAllDrivesBut(driveCircle); driveCircle.start();    RobotHardware.outtakeAngleAdjust.setPosition(MConstants.flapDown);}
        if (gamepad1.squareWasPressed())   { outtakeSpeed = 1050; cancelAllDrivesBut(driveSquare); driveSquare.start();    RobotHardware.outtakeAngleAdjust.setPosition(MConstants.flapUp);}
        if (gamepad1.triangleWasPressed()) { outtakeSpeed = 1590; cancelAllDrivesBut(driveTriangle); driveTriangle.start();  RobotHardware.outtakeAngleAdjust.setPosition(MConstants.flapDown);}
        if (gamepad1.crossWasPressed())    { outtakeSpeed = 1150; cancelAllDrivesBut(driveCross); driveCross.start();     RobotHardware.outtakeAngleAdjust.setPosition(MConstants.flapUp);}
        if (gamepad1.touchpadWasPressed()) {                      cancelAllDrivesBut(drivePark); drivePark.start(); }

        // ----------------------------
        // AUTO-FIRE BUTTON
        // ----------------------------
        if (gamepad1.right_bumper) {
            if (fireTask == null) {
                fireTask = new AutoFireTask(outtake, indexer, ejector, intake, outtakeSpeed);
                fireTask.start();
            }
        } else {
            if (fireTask != null) {
                fireTask.cancel();
            }
        }
        if (gamepad1.optionsWasPressed()) {
            if (driveCircle.isActive())   driveCircle.cancel();
            if (driveSquare.isActive())   driveSquare.cancel();
            if (driveTriangle.isActive()) driveTriangle.cancel();
            if (driveCross.isActive())    driveCross.cancel();
            if (drivePark.isActive())     drivePark.cancel();
        }

        // ----------------------------
        // UPDATE AUTO TASKS
        // ----------------------------
        driveCircle.update();
        driveSquare.update();
        driveTriangle.update();
        driveCross.update();
        drivePark.update();

        if (fireTask != null) {
            fireTask.update();
            if (!fireTask.isActive()) fireTask = null;
        }

        // ----------------------------
        // TELEMETRY
        // ----------------------------
        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetry.addData("Slow Mode %", slowModeMultiplier);
        telemetry.addData("Slow Mode Active", slowMode);
        telemetry.addData("Outtake Active", outtakeON);
        telemetry.addData("Outtake Speed", outtakeSpeed);
        telemetry.update();
    }

    private void cancelAllDrives() {
        driveCircle.cancel();
        driveSquare.cancel();
        driveTriangle.cancel();
        driveCross.cancel();
        drivePark.cancel();
    }
    private void cancelAllDrivesBut(AutoDriveTask exclude) {
        if (exclude != driveCircle) {
            driveCircle.cancel();
        }
        if (exclude != driveSquare) {
            driveSquare.cancel();
        }
        if (exclude != driveTriangle) {
            driveTriangle.cancel();
        }
        if (exclude != driveCross) {
            driveCross.cancel();
        }
        if (exclude != drivePark) {
            drivePark.cancel();
        }
    }

    private boolean anyDriveTaskActive() {
        return driveCircle.isActive() || driveSquare.isActive() || driveTriangle.isActive() || driveCross.isActive() || drivePark.isActive();
    }

    private boolean autoFireActive() {
        return fireTask != null && fireTask.isActive();
    }

}
