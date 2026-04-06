package org.firstinspires.ftc.teamcode.teleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.AdafruitNeoPixel;
import org.firstinspires.ftc.teamcode.hardware.Color;
import java.util.Arrays;

// CONSTANTS
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.PixelController;
import org.firstinspires.ftc.teamcode.pedroPathing.AlliancePoseProvider;
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
    private Pose goalPose;
    AlliancePoseProvider poses;

    private PixelController pixel;

    @Override
    public void init() {
        RobotHardware.init(hardwareMap);
        Follower follower = Constants.createFollower(hardwareMap);

        poses = new AlliancePoseProvider(alliance);

        startPose = poses.get(new Pose(112.5, 18.5, Math.toRadians(0))); //FAR: 12.5, 18.5, Math.toRadians(0)    //LEAVE: 112.5, 13.5, Math.toRadians(0)    //CLOSE: 96,109,Math.toRadians(45)

        driveTask = new DriveTask(
                follower,
                poses,
                startPose
        );

        goalPose = poses.get(MConstants.goalPoseRed);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        intake = new IntakeAction(hardwareMap);
        indexer = new IndexAction(hardwareMap);
        ejector = new EjectorAction(hardwareMap);
        blocker = new BlockerAction(hardwareMap);
        outtake = new OuttakeAction(hardwareMap);
        turret = new TurretAction(hardwareMap);

        ballSensors = new BallSensorArray();

        aimTask = new ShooterAimTask(goalPose, ShooterAimTask.speedMap());

        pixel = new PixelController(alliance);
        pixel.setAllianceColor();
    }

    @Override
    public void init_loop() {
        pixel.show();
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
        if (gamepad1.psWasPressed()) should_turret_track_target = !should_turret_track_target;

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

        // ----- FLAP -----
        if (aimTask.getDistanceToGoal(currentPose) > MConstants.distToOpen) RobotHardware.outtakeAngleAdjust.setPosition(MConstants.flapDown);
        else RobotHardware.outtakeAngleAdjust.setPosition(MConstants.flapUp);

        // ----- Auto-fire (right bumper): sensor-based when ballSensors available -----
        if (gamepad1.right_bumper) {
            blocker.in();
            if (fireTask == null) {
                fireTask = new AutoFireTask(outtake, indexer, ejector, intake, ballSensors, outtakeSpeed, 3);
                fireTask.start();
            }
        } else {
            if (fireTask != null) fireTask.cancel();

            blocker.out();

            // ----- intake -----
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


            // ----- indexer -----
            if (gamepad1.right_trigger > 0.25) {
                indexer.runIn();
            } else {
                indexer.stop();
            }
        }
        if (fireTask != null) {
            fireTask.update(outtakeSpeed);
            if (!fireTask.isActive()) fireTask = null;
        }

        pixel.ballSensorTele(ballSensors.getBallCount(), ballSensors.hasBallAt(BallPosition.LAUNCH_READY));

        // ----- Auto-aim: shooter at distance-based speed; one button turns to face target -----
        outtakeSpeed = aimTask.getTargetSpeed(currentPose);
        outtake.spinUp(outtakeSpeed);
        outtake.update(); // Run shooter PID every loop for stable speed

        // ----- Telemetry -----
        telemetryM.debug("position", driveTask.getPose());
        telemetryM.debug("velocity", driveTask.getVelocity());

        telemetry.addData("Pos of base", "X: %.2f, Y: %.2f, H: %.2f°",
                driveTask.getPose().getX(),
                driveTask.getPose().getY(),
                Math.toDegrees(driveTask.getPose().getHeading())
        );

        telemetry.addData("Alliance", alliance);

        telemetry.addData("Outtake Speed (target)", outtakeSpeed);
        telemetry.addData("Outtake Speed (real)", outtake.motor.getVelocity());
        telemetry.addData("Distance to goal", aimTask.getDistanceToGoal(currentPose));

        telemetry.addData("turret target Position", turret.controller.getTargetPosition());
        telemetry.addData("real turret position", turret.motor.getCurrentPosition());
        telemetry.addData("turret Error", turret.controller.getTargetPosition() - turret.motor.getCurrentPosition());

        telemetry.addData("Current draw", "Base: %.2f, Intake: %.2f, Turret: %.2f°, Outtake: %.2f",
                driveTask.getCurrentDraw(),
                intake.getCurrentDraw(),
                turret.getCurrentDraw(),
                outtake.getCurrentDraw()
        );

        telemetry.update();
    }
}
