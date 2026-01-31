package org.firstinspires.ftc.teamcode.pedroPathing.Teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.tools.ButtonHandler;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.MConstants;

import org.firstinspires.ftc.teamcode.tools.Actions.AutoFireTask;

import org.firstinspires.ftc.teamcode.tools.Actions.BlockerAction;
import org.firstinspires.ftc.teamcode.tools.Actions.EjectorAction;
import org.firstinspires.ftc.teamcode.tools.Actions.IndexAction;
import org.firstinspires.ftc.teamcode.tools.Actions.IntakeAction;
import org.firstinspires.ftc.teamcode.tools.Actions.OuttakeAction;
import org.firstinspires.ftc.teamcode.tools.Actions.TurretAction;

import org.firstinspires.ftc.teamcode.tools.Sensors.BallPosition;
import org.firstinspires.ftc.teamcode.tools.Sensors.BallSensorArray;

@Configurable
@TeleOp
public class BLUE_FEEDBACK extends OpMode {

    // ----- Drive -----
    private Follower follower;
    public static Pose startingPose = new Pose(144-127, 87, Math.toRadians(180-0));
    private double fieldCentricOffset = startingPose.getHeading();
    private final double k = 2.0;
    private final double expKMinus1 = Math.exp(k) - 1;
    private boolean slowMode = false;
    private final double slowModeMultiplier = 0.25;

    // ----- Telemetry -----
    private TelemetryManager telemetryM;

    // ----- Button handlers -----
    private final ButtonHandler outtakeVelocityUp = new ButtonHandler();
    private final ButtonHandler outtakeVelocityDown = new ButtonHandler();
    private final ButtonHandler intakeDirUp = new ButtonHandler();
    private final ButtonHandler intakeDirDown = new ButtonHandler();
    private final ButtonHandler turretTrackToggle = new ButtonHandler();

    // ----- Subsystems -----
    private IntakeAction intake;
    private IndexAction indexer;
    private EjectorAction ejector;
    private BlockerAction blocker;
    private OuttakeAction outtake;
    private TurretAction turret;
    private BallSensorArray ballSensors;

    private long outtakeSpeed = 1000;
    private final long outtakeSpeedMin = 500;
    private final long outtakeSpeedMax = 3000;
    private double intakePow = 0;

    private boolean turret_track_target = true;

    // ----- Auto-aim / Shooter -----
    private ShooterAimHelper aimHelper;
    private AutoFireTask fireTask = null;

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
        blocker = new BlockerAction(hardwareMap);
        outtake = new OuttakeAction(hardwareMap);
        turret = new TurretAction(hardwareMap);

        ballSensors = new BallSensorArray();

        aimHelper = ShooterAimHelper.defaultBlue(); // MAKE SURE THIS IS THE PROPER SIDE AFTER MIRROR!

        outtakeVelocityUp.setOnPress(() -> RobotHardware.outtakeAngleAdjust.setPosition(MConstants.flapDown));
        outtakeVelocityDown.setOnPress(() -> RobotHardware.outtakeAngleAdjust.setPosition(MConstants.flapUp));

        intakeDirDown.setOnPress(() -> intakePow = (intakePow > 0 ? 0 : MConstants.intakePowerOut));
        intakeDirUp.setOnPress(() -> intakePow = (intakePow < 0 ? 0 : MConstants.intakePowerIn));

        turretTrackToggle.setOnPress(() -> turret_track_target = !turret_track_target);

        fieldCentricOffset = follower.getHeading();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        outtake.spinUp(outtakeSpeed);
        ejector.down();
        RobotHardware.outtakeAngleAdjust.setPosition(MConstants.flapDown);
        fieldCentricOffset = Math.toRadians(180-0);
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();

        // ----- Buttons -----
        outtakeVelocityUp.update(gamepad1.dpad_up);
        outtakeVelocityDown.update(gamepad1.dpad_down);

        intakeDirUp.update(gamepad1.left_bumper);
        intakeDirDown.update(gamepad1.left_trigger, 0.25F);

        turretTrackToggle.update(gamepad1.right_trigger, 0.25f);

        slowMode = gamepad1.left_stick_button;


        // ----- Get robot location
        Pose currentPose = follower.getPose();
        double heading = currentPose.getHeading() - fieldCentricOffset;

        // ----- Drive inputs -----
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x / 2;

        // ----- Auto-aim: shooter at distance-based speed; one button turns to face target -----
        outtakeSpeed = aimHelper.getTargetSpeed(currentPose);
        outtake.spinUp(outtakeSpeed);
        outtake.update(); // Run shooter PID every loop for stable speed

//        boolean autoAimActive = gamepad1.right_trigger > 0.5; // disabled the auto aim
//        if (autoAimActive) {
//            turn = aimHelper.getTurnCorrection(currentPose);
//            telemetry.addData("Auto-Aim", "ACTIVE");
//        } else {
//            aimHelper.resetHeadingPID(); // Clear integral when not aiming
//        }
        Pose goalPose = aimHelper.getGoalPose();
//        double targetAngle = Math.atan2(
//                goalPose.getX() - currentPose.getX(),
//                -(goalPose.getY() - currentPose.getY()));



        double xDiff = (goalPose.getX() - currentPose.getX());
        double yDiff = (goalPose.getY() -  currentPose.getY());
//        double compDistance = Math.hypot(xDiff, yDiff);



        // Iterative compensation


        double angleToTarget = Math.atan2(yDiff, xDiff);
        double targetAngle = angleToTarget;
        double turretError = (angleToTarget - heading) + Math.PI;
        turretError = Math.atan2(Math.sin(turretError), Math.cos(turretError));

        if (turret_track_target) {
            turret.runToTick((int) (turretError * MConstants.MAGIC));
        } else {
            turret.runToTick(turret.center); // hold at center 0
        }
       // double targetAngle = Math.atan2(
             //   (goalPose.getY() - currentPose.getY()),
                //(goalPose.getX() - currentPose.getX()));

//        double wrappedAngle = turret.calculateWrapAngles(targetAngle);
//        double limitedAngle = turret.limitRadValues(wrappedAngle);

        //int tickRaw = turret.radToTick(limitedAngle);

       // turret.runToTick(tickRaw);

        if (ballSensors.hasBallAt(BallPosition.FUNNEL_LEFT) || ballSensors.hasBallAt(BallPosition.FUNNEL_RIGHT)) { // if detect ball go into robot rumble
            gamepad1.rumble(0.4, 0.4, 10);
        } else {
            gamepad1.stopRumble();
        }

        // ----- Mecanum drive -----
        double driveY = Math.signum(y) * (Math.exp(k * Math.abs(y)) - 1) / expKMinus1;
        double driveX = Math.signum(x) * (Math.exp(k * Math.abs(x)) - 1) / expKMinus1;
        if (slowMode) {
            driveX *= slowModeMultiplier;
            driveY *= slowModeMultiplier;
            turn *= slowModeMultiplier;
        }
        double rotX = driveX * Math.cos(-heading) - driveY * Math.sin(-heading);
        double rotY = driveX * Math.sin(-heading) + driveY * Math.cos(-heading);
        double denom = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
        RobotHardware.frontLeftMotor.setPower((rotY + rotX + turn) / denom);
        RobotHardware.backLeftMotor.setPower((rotY - rotX + turn) / denom);
        RobotHardware.frontRightMotor.setPower((rotY - rotX - turn) / denom);
        RobotHardware.backRightMotor.setPower((rotY + rotX - turn) / denom);

        // ----- Subsystems -----
        if (gamepad1.psWasPressed()) {
            follower.setPose(new Pose(144-120, 127, Math.toRadians(180-37)));
            fieldCentricOffset = startingPose.getHeading();
        }
        if (gamepad1.optionsWasPressed()) {
            follower.setPose(new Pose(144-7.5, 8, Math.toRadians(180-0)));
            fieldCentricOffset = Math.toRadians(180-0);
        }
        if (gamepad1.shareWasPressed()) fieldCentricOffset = follower.getHeading();

        // ----- Auto-fire (right bumper): sensor-based when ballSensors available -----
        if (gamepad1.right_bumper) {
            blocker.in();
            if (fireTask == null) {
                fireTask = new AutoFireTask(outtake, indexer, ejector, intake, ballSensors, outtakeSpeed);
                fireTask.start();
            }
        } else {
            if (fireTask != null) fireTask.cancel();

            blocker.out();

            if (intakePow == MConstants.intakePowerIn) intake.runIn();
            else if (intakePow == MConstants.intakePowerOut) intake.runOut();
            else intake.stop();
        }
        if (fireTask != null) {
            fireTask.update();
            if (!fireTask.isActive()) fireTask = null;
        }

        // ----- Telemetry -----
        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetry.addData("Slow Mode %", slowModeMultiplier);
        telemetry.addData("Slow Mode Active", slowMode);
        telemetry.addData("Outtake Speed (target)", outtakeSpeed);
        telemetry.addData("Outtake Speed (real)", outtake.motor.getVelocity());
        telemetry.addData("Distance", aimHelper.getDistanceToGoal(currentPose));
        telemetry.addData("Angle", currentPose.getHeading());


        telemetry.addData("targetAngle angle", targetAngle);
//        telemetry.addData("joystick angle wrapped", wrappedAngle);
//        telemetry.addData("wrapped angle limited", limitedAngle);
        telemetry.addData("Offset", fieldCentricOffset);
        telemetry.addData("tick target raw", (turretError * MConstants.MAGIC));

        telemetry.addData("turret target Position", turret.motor.getTargetPosition());
        telemetry.addData("real turret position", turret.motor.getCurrentPosition());

        for (BallPosition p : BallPosition.values()) { // for each sensor of the enum in ball position
            telemetry.addData(
                p.name(),
                ballSensors.hasBallAt(p) + " (" + ballSensors.getDistanceCm(p) + "cm)"
            );
        }


        telemetry.update();
    }
}
