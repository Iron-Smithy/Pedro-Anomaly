package org.firstinspires.ftc.teamcode.pedroPathing.Teleop;

// pedro, FTC, Not custom etc
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.MConstants;
//import org.firstinspires.ftc.teamcode.tools.Actions.AutoDriveTaskEX;
import org.firstinspires.ftc.teamcode.tools.Actions.AutoFireTask;
import org.firstinspires.ftc.teamcode.tools.Actions.EjectorAction;
import org.firstinspires.ftc.teamcode.tools.Actions.IndexAction;
import org.firstinspires.ftc.teamcode.tools.Actions.IntakeAction;
import org.firstinspires.ftc.teamcode.tools.Actions.OuttakeAction;
import org.firstinspires.ftc.teamcode.tools.ButtonHandler;

import java.util.Map;
import java.util.TreeMap;
import java.util.function.Supplier;

@Configurable
@TeleOp
public class PDriveA_BLUE_EX_0 extends OpMode {

    /* DRIVE SYSTEM */
    private Follower follower;
    public static Pose startingPose = new Pose(144-127, 87, Math.toRadians(180-0));//new Pose(144-120, 127, Math.toRadians(180-37)); //new Pose(120, 127, Math.toRadians(37));

    // Field-centric
    private double fieldCentricOffset = startingPose.getHeading();

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
//    private AutoDriveTaskEX driveCircle;
//    private AutoDriveTaskEX driveSquare;
//    private AutoDriveTaskEX driveTriangle;
//    private AutoDriveTaskEX driveCross;
//    private AutoDriveTaskEX drivePark;
    private AutoFireTask fireTask = null;

    private Supplier<PathChain> park;

    // experimetnal ====================================================================================================================================================================================
    public static final Pose GOAL_POSE = new Pose(144-132, 136, 180-0);
    public static double TURN_P_Close = 0.09; // Increase if too slow, decrease if oscillating
    public static double TURN_P_Far = 0.045; // Increase if too slow, decrease if oscillating
    private TreeMap<Double, Long> speedMap = new TreeMap<>();

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

        outtakeVelocityUp.setOnPress(() -> RobotHardware.outtakeAngleAdjust.setPosition(MConstants.flapDown));
        outtakeVelocityDown.setOnPress(() -> RobotHardware.outtakeAngleAdjust.setPosition(MConstants.flapUp));
        intakeDirDown.setOnPress(() -> intakePow = (intakePow > 0 ? 0 : MConstants.intakePowerOut));
        intakeDirUp.setOnPress(() -> intakePow = (intakePow < 0 ? 0 : MConstants.intakePowerIn));

        fieldCentricOffset = follower.getHeading();

        // Initialize auto-drive tasks
//        driveCircle   = new AutoDriveTaskEX(follower, new Pose(84, 84, Math.toRadians(47)));
//        driveSquare   = new AutoDriveTaskEX(follower, new Pose(108, 108, Math.toRadians(47)));
//        driveTriangle = new AutoDriveTaskEX(follower, new Pose(84, 20, Math.toRadians(65)));
//        driveCross    = new AutoDriveTaskEX(follower, new Pose(84, 132, Math.toRadians(7)));
//        drivePark     = new AutoDriveTaskEX(follower, new Pose (38.6, 40, Math.toRadians(90)));

        // experimetnal ====================================================================================================================================================================================
        // Format: speedMap.put(DistanceInches, MotorRPM);

        // Point 1: Right against the sub/wall
        speedMap.put(12.0 * 1, 850L); // TUNE ME!!!

        // Point 2: A normal shooting distance
        speedMap.put(12.0 * 5, 1130L); // TUNE ME!!!

        // Point 3: Mid-field
        speedMap.put(12.0 * 9, 1200L); // TUNE ME!!!

        // Point 4: Far shot
        speedMap.put(12.0 * 12, 1640L); // TUNE ME!!!

        // Point 5: Far-est shot
        speedMap.put(12.0 * 14, 1680L); // TUNE ME!!!
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        outtake.spinUp(outtakeSpeed);
        ejector.down();
//        outtake.stop();

        RobotHardware.outtakeAngleAdjust.setPosition(MConstants.flapDown);

//        follower.setPose(startingPose);
        fieldCentricOffset = startingPose.getHeading();
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();

        // ----------------------------
        // MANUAL DRIVE
        // ----------------------------
        if (true) {//!anyDriveTaskActive()) {
            Pose currentPose = follower.getPose();
            double heading = currentPose.getHeading() - fieldCentricOffset;
            // double heading = follower.getHeading() - fieldCentricOffset;

            // 1. Standard Driver Inputs
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x / 2;

            // 2. AUTO-AIM
            // experimetnal ====================================================================================================================================================================================
            // Use Right Trigger for aiming (x > 50%)
            boolean autoAimActive = gamepad1.right_trigger > 0.5;

            double distToGoal = Math.hypot(GOAL_POSE.getX() - currentPose.getX(), GOAL_POSE.getY() - currentPose.getY());
            outtakeSpeed = getInterpolatedSpeed(distToGoal);
            new AutoFireTask(outtake, indexer, ejector, intake, outtakeSpeed).spinUp(outtakeSpeed);

            if (autoAimActive) {
                // A. Calculate the target angle using Trig (atan2)
                // atan2(dy, dx) gives the absolute angle to the target

                double targetAngle = Math.atan2(GOAL_POSE.getY() - currentPose.getY(), GOAL_POSE.getX() - currentPose.getX());
                
                // B. IMPORTANT: Pedro Pathing heading might need an offset depending on your robot's "front"
                // If your intake is the front, this is fine.
                // If your shooter is on the back, add Math.PI (180 deg) to targetAngle.
                 targetAngle += Math.PI;

                // C. Calculate error and apply P-Control
                double error = angleWrap(targetAngle - currentPose.getHeading());
                
                // Override the manual 'turn' variable
                // We use the negative sign if the robot turns the wrong way
                if (distToGoal < (8 * 12)) {
                    turn = error * TURN_P_Close;
                } else {
                    turn = error * TURN_P_Far;
                }

                // D. Set Speed from the Map
//                outtakeSpeed = getInterpolatedSpeed(distToGoal);

                telemetry.addData("Auto-Aim", "ACTIVE");
                telemetry.addData("Target Angle", Math.toDegrees(targetAngle));
                telemetry.addData("Err", Math.toDegrees(error));
            }

            // 3. Apply Powers (Pedro / Mecanum logic)
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
                follower.setPose(new Pose(144-120, 127, Math.toRadians(180-37)));
                fieldCentricOffset = startingPose.getHeading();
//                fieldCentricOffset = follower.getHeading();
            }
            if (gamepad1.optionsWasPressed()) {
                follower.setPose(new Pose(144-8, 8, Math.toRadians(180-0)));
                fieldCentricOffset = follower.getHeading();
            }
        }

        if (gamepad1.shareWasPressed()) fieldCentricOffset = follower.getHeading();

        // ----------------------------
        // AUTO-DRIVE BUTTONS
        // ----------------------------
//        if (gamepad1.circle)    { outtakeSpeed = 1200; cancelAllDrivesBut(driveCircle);   driveCircle.start();   RobotHardware.outtakeAngleAdjust.setPosition(MConstants.flapDown);}
//        else if (driveCircle.isActive())    { driveCircle.cancel();}
//
//        if (gamepad1.square)    { outtakeSpeed = 1050; cancelAllDrivesBut(driveSquare);   driveSquare.start();   RobotHardware.outtakeAngleAdjust.setPosition(MConstants.flapUp);}
//        else if (driveSquare.isActive())    { driveCircle.cancel();}
//
//        if (gamepad1.triangle)  { outtakeSpeed = 1590; cancelAllDrivesBut(driveTriangle); driveTriangle.start(); RobotHardware.outtakeAngleAdjust.setPosition(MConstants.flapDown);}
//        else if (driveTriangle.isActive())  { driveCircle.cancel();}
//
//        if (gamepad1.cross)     { outtakeSpeed = 1150; cancelAllDrivesBut(driveCross);    driveCross.start();    RobotHardware.outtakeAngleAdjust.setPosition(MConstants.flapUp);}
//        else if (driveCross.isActive())     { driveCircle.cancel();}
//
//        if (gamepad1.touchpad)  { cancelAllDrivesBut(drivePark);     drivePark.start(); }
//        else if (drivePark.isActive())      { driveCircle.cancel();}

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
//        if (gamepad1.optionsWasPressed()) {
//            if (driveCircle.isActive())   driveCircle.cancel();
//            if (driveSquare.isActive())   driveSquare.cancel();
//            if (driveTriangle.isActive()) driveTriangle.cancel();
//            if (driveCross.isActive())    driveCross.cancel();
//            if (drivePark.isActive())     drivePark.cancel();
//        }

        // ----------------------------
        // UPDATE AUTO TASKS
        // ----------------------------
//        driveCircle.update();
//        driveSquare.update();
//        driveTriangle.update();
//        driveCross.update();
//        drivePark.update();

        if (fireTask != null) {
            fireTask.update();
            if (!fireTask.isActive()) fireTask = null;
        }

        // ----------------------------
        // TELEMETRY
        // ----------------------------
        Pose currentPose = follower.getPose();
        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetry.addData("Slow Mode %", slowModeMultiplier);
        telemetry.addData("Slow Mode Active", slowMode);
        telemetry.addData("Outtake Active", outtakeON);
        telemetry.addData("Outtake Speed", outtakeSpeed);
        telemetry.addData("Distance", Math.hypot(GOAL_POSE.getX() - currentPose.getX(), GOAL_POSE.getY() - currentPose.getY()));
        telemetry.addData("Angle", currentPose.getHeading());
        telemetry.addData("Offset", fieldCentricOffset);
        telemetry.addData("PosX", Math.round(currentPose.getX()));
        telemetry.addData("PosY", Math.round(currentPose.getY()));
        telemetry.update();
    }

//    private void cancelAllDrives() {
//        driveCircle.cancel();
//        driveSquare.cancel();
//        driveTriangle.cancel();
//        driveCross.cancel();
//        drivePark.cancel();
//    }
//    private void cancelAllDrivesBut(AutoDriveTaskEX exclude) {
//        if (exclude != driveCircle) {
//            driveCircle.cancel();
//        }
//        if (exclude != driveSquare) {
//            driveSquare.cancel();
//        }
//        if (exclude != driveTriangle) {
//            driveTriangle.cancel();
//        }
//        if (exclude != driveCross) {
//            driveCross.cancel();
//        }
//        if (exclude != drivePark) {
//            drivePark.cancel();
//        }
//    }
//
//    private boolean anyDriveTaskActive() {
//        return driveCircle.isActive() || driveSquare.isActive() || driveTriangle.isActive() || driveCross.isActive() || drivePark.isActive();
//    }

    private boolean autoFireActive() {
        return fireTask != null && fireTask.isActive();
    }
    // experimetnal ====================================================================================================================================================================================
    /**
     * Looks up the distance in the table and interpolates the RPM.
     */
    private long getInterpolatedSpeed(double dist) {
        // Handle out of bounds (too close or too far)
        if (dist <= speedMap.firstKey()) return speedMap.firstEntry().getValue();
        if (dist >= speedMap.lastKey()) return speedMap.lastEntry().getValue();

        // Find the two points surrounding our current distance
        Map.Entry<Double, Long> low = speedMap.floorEntry(dist);
        Map.Entry<Double, Long> high = speedMap.ceilingEntry(dist);

        if (low == null || high == null) return 2000; // Fail-safe

        // Linear Interpolation formula: y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
        double distanceRange = high.getKey() - low.getKey();
        double speedRange = high.getValue() - low.getValue();
        double fraction = (dist - low.getKey()) / distanceRange;

        return (long) (low.getValue() + (fraction * speedRange));
    }
    public double angleWrap(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }
}