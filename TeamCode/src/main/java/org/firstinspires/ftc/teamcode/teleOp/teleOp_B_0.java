package org.firstinspires.ftc.teamcode.teleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

// CONSTANTS
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Auton.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.MConstants;

// ACTIONS
import org.firstinspires.ftc.teamcode.Actions.BlockerAction;
import org.firstinspires.ftc.teamcode.Actions.EjectorAction;
import org.firstinspires.ftc.teamcode.Actions.IndexAction;
import org.firstinspires.ftc.teamcode.Actions.IntakeAction;
import org.firstinspires.ftc.teamcode.Actions.OuttakeAction;
import org.firstinspires.ftc.teamcode.Actions.TurretAction;

// SENSORS
import org.firstinspires.ftc.teamcode.Sensors.BallPosition;
import org.firstinspires.ftc.teamcode.Sensors.BallSensorArray;

// TASKS
import org.firstinspires.ftc.teamcode.Tasks.DriveTask;
import org.firstinspires.ftc.teamcode.Tasks.ShooterAimTask;
import org.firstinspires.ftc.teamcode.Tasks.AutoFireTask;

@Configurable
public class teleOp_B_0 extends OpMode {
    private Alliance alliance = Alliance.RED; // defualt

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    private DriveTask driveTask;

    // ----- Telemetry -----
    private TelemetryManager telemetryM;

    // ----- Subsystems -----
    private IntakeAction intake;
    private IndexAction indexer;
    private EjectorAction ejector;
    private BlockerAction blocker;
    private OuttakeAction outtake;
    private TurretAction turret;
    private BallSensorArray ballSensors;

    private long outtakeSpeed = 1000;

    private boolean should_turret_track_target = true;

    // ----- Auto-aim / Shooter -----
    private ShooterAimTask aimTask;
    private AutoFireTask fireTask = null;

    // ---- INTAKE STUFF
    ElapsedTime triggerTimer = new ElapsedTime();
    boolean lastTrigger = false;
    final double PULSE_MS = 1;
    final double HOLD_DELAY_MS = 500;

    private Pose startPose;
    private Pose goalRESET;
    private Pose humanRESET;
    private Pose goalPose;

    private void doPoseMath() {
         startPose = pose(MConstants.startPoseRed);
         goalRESET = pose(MConstants.goalResetPoseRed);
         humanRESET = pose(MConstants.humanPlayerPoseRed);
         goalPose = pose(MConstants.goalPoseRed);
    }
    private Pose pose(Pose redPose) {
        if (alliance == Alliance.RED) {
            return redPose;
        } else {
            return redPose.mirror();
        }
    }

    @Override
    public void init() {
        RobotHardware.init(hardwareMap);
        Follower follower = Constants.createFollower(hardwareMap);

        doPoseMath();

        driveTask = new DriveTask(startPose, follower, goalRESET, humanRESET, alliance);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        intake = new IntakeAction(hardwareMap);
        indexer = new IndexAction(hardwareMap);
        ejector = new EjectorAction(hardwareMap);
        blocker = new BlockerAction(hardwareMap);
        outtake = new OuttakeAction(hardwareMap);
        turret = new TurretAction(hardwareMap);

        ballSensors = new BallSensorArray();

        aimTask = new ShooterAimTask(goalPose, ShooterAimTask.speedMap());
    }

    @Override
    public void start() {
//        driveTask.startTeleOp();
        outtake.spinUp(outtakeSpeed);
        ejector.down();
        RobotHardware.outtakeAngleAdjust.setPosition(MConstants.flapDown);
        driveTask.startTeleOp();
    }

    @Override
    public void loop() {
        driveTask.update(gamepad1);
        Pose currentPose = driveTask.getPose();
        telemetryM.update();

        // ----- Buttons -----
        if (gamepad1.dpad_up) RobotHardware.outtakeAngleAdjust.setPosition(MConstants.flapDown);
        else if (gamepad1.dpad_down) RobotHardware.outtakeAngleAdjust.setPosition(MConstants.flapUp);
//        if () RobotHardware.outtakeAngleAdjust.setPosition(MConstants.flapDown);
//        else RobotHardware.outtakeAngleAdjust.setPosition(MConstants.flapUp);

        if (gamepad1.triangleWasPressed()) should_turret_track_target = !should_turret_track_target;

        // ----- Auto-aim: shooter at distance-based speed; one button turns to face target -----
        outtakeSpeed = aimTask.getTargetSpeed(currentPose);
        outtake.spinUp(outtakeSpeed);
        outtake.update(); // Run shooter PID every loop for stable speed

        Pose goalPose = aimTask.getGoalPose();
        double xDiff = (goalPose.getX() - currentPose.getX());
        double yDiff = (goalPose.getY() -  currentPose.getY());
        double targetAngle = Math.atan2(yDiff, xDiff);
        double turretError = targetAngle - driveTask.getHeading();
        double wrappedAngle = turret.calculateWrapAngles(turretError);
        double limitedAngle = turret.limitRadValues(wrappedAngle);
        int tickRaw = turret.radToTick(limitedAngle);
        if (should_turret_track_target) {
            turret.runToTick(tickRaw);
        } else {
            turret.runToTick(turret.center); // hold at center 0
        }

        if (ballSensors.hasBallAt(BallPosition.FUNNEL_LEFT) || ballSensors.hasBallAt(BallPosition.FUNNEL_RIGHT)) { // if detect ball go into robot rumble
            gamepad1.rumble(0.4, 0.4, 10);
        } else {
            gamepad1.stopRumble();
        }

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

            // ----- intake
//            if (gamepad1.left_bumper) intake.runIn(); // old code before pulse
//            else if (gamepad1.left_trigger > 0.25) {
//                intake.runOut();
//            } else intake.stop();

            boolean currentTrigger = gamepad1.left_trigger > 0.25;
            if (gamepad1.left_bumper) {
                intake.runIn();
            } else if (currentTrigger) {
                // If this is the very first frame he pressed it, reset the timer
                if (!lastTrigger) triggerTimer.reset();

                double ms = triggerTimer.milliseconds();

                if (ms < PULSE_MS) {
                    intake.runOut();
                } else if (ms < HOLD_DELAY_MS) {
                    intake.stop();
                } else {
                    intake.runOut();
                }
            } else {
                intake.stop();
            }
            lastTrigger = currentTrigger;


            // ----- indexer
            if (gamepad1.right_trigger > 0.25) {
                indexer.runIn();
            } else {
                indexer.stop();
            }
        }
        if (fireTask != null) {
            fireTask.update();
            if (!fireTask.isActive()) fireTask = null;
        }

        // ----- Telemetry -----
        telemetryM.debug("position", driveTask.getPose());
        telemetryM.debug("velocity", driveTask.getVelocity());

        telemetry.addData("Pos of base", "X: %.2f, Y: %.2f, H: %.2f°",
                driveTask.getPose().getX(),
                driveTask.getPose().getY(),
                Math.toDegrees(driveTask.getPose().getHeading())
        );

        telemetry.addData("Alliance", alliance);
        telemetry.addData("Is Red Alliance", alliance == Alliance.RED);
        telemetry.addData("Goal Pose", goalPose);
        telemetry.addData("Start Pose", startPose);

        telemetry.addData("turret error", turretError);

        telemetry.addData("Outtake Speed (target)", outtakeSpeed);
        telemetry.addData("Outtake Speed (real)", outtake.motor.getVelocity());
        telemetry.addData("Distance to goal", aimTask.getDistanceToGoal(currentPose));

        telemetry.addData("targetAngle angle (robot centric)", Math.toDegrees(limitedAngle));
        telemetry.addData("targetTick", tickRaw);
        telemetry.addData("turret target Position", turret.controller.getTargetPosition());
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
