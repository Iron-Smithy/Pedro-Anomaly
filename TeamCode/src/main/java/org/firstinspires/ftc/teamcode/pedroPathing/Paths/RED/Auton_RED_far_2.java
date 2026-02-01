// untested 12 piece auto
package org.firstinspires.ftc.teamcode.pedroPathing.Paths.RED;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.MConstants;
import org.firstinspires.ftc.teamcode.tools.Actions.AutoFireTask;
import org.firstinspires.ftc.teamcode.tools.Actions.EjectorAction;
import org.firstinspires.ftc.teamcode.tools.Actions.IndexAction;
import org.firstinspires.ftc.teamcode.tools.Actions.IntakeAction;
import org.firstinspires.ftc.teamcode.tools.Actions.OuttakeAction;
import org.firstinspires.ftc.teamcode.tools.Actions.BlockerAction;
import org.firstinspires.ftc.teamcode.tools.Actions.TurretAction;

import org.firstinspires.ftc.teamcode.tools.Sensors.BallSensorArray;

@Autonomous(name = "Auton_RED_far", group = "AAA")
public class Auton_RED_far_2 extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    private IntakeAction intake;
    private IndexAction indexer;
    private EjectorAction ejector;
    private OuttakeAction outtake;
    private BlockerAction blocker;
    private TurretAction turret;

    private BallSensorArray ballSensors;

    private long scoreShooterTPS = 1444;

    private long tolerance = 25;

    private AutoFireTask fireTask = null;
    private final Pose startPose = new Pose(-48+144, 8, Math.toRadians(-90+180)); // start location // r = -b + (144 || 180)  || b = 144 - r (simplify later)====================================================
    private final Pose ScorePose = new Pose(-54+144, 13, Math.toRadians(-109+180)); // score location
    private final Pose R3PrePose = new Pose(-45+144, 39, Math.toRadians(-180+180)); // row 1 collection pre location
    private final Pose R3CollectPose = new Pose(-9+144, 39, Math.toRadians(-180+180)); // row 1 balls inside robot location
    private final Pose RHuman = new Pose(-9+144, 13, Math.toRadians(-180+180)); // smooth back-out bezier control point

    private enum AutoState {
        START,
        GO_SCORE_PRELOAD,
        SCORE_PRELOAD,

        PICKUP_R3, // row 3
        GO_SCORE_R3,
        SCORE_R3,

        PICKUP_RHuman, // row H
        GO_SCORE_RHuman,
        SCORE_RHuman,

        EXIT,
        DONE
    }

    private AutoState currentState = AutoState.START;

    private PathChain startToScore;
    private PathChain row3ToScore;
    private PathChain row3Pickup;
    private PathChain rowHPickup;
    private PathChain rowHToScore;
    private PathChain scoreToPark;

    private void buildPaths() {
        // ========= START → SCORE =========
        startToScore = follower.pathBuilder()
                .addPath(new BezierLine(startPose, ScorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), ScorePose.getHeading())
                .setVelocityConstraint(0.5)
                .build();

        // ========= ROW 1 =========
        row3Pickup = follower.pathBuilder()
                .addPath(new BezierLine(ScorePose, R3PrePose))
                .setLinearHeadingInterpolation(ScorePose.getHeading(), R3PrePose.getHeading())
                .addPath(new BezierLine(R3PrePose, R3CollectPose))
                .setConstantHeadingInterpolation(R3CollectPose.getHeading())
                .setVelocityConstraint(0.1)
                .build();

        row3ToScore = follower.pathBuilder()
                // You can still use control points by using the addPath method that takes multiple Poses
                .addPath(new BezierLine(R3CollectPose, ScorePose))
                .setLinearHeadingInterpolation(R3CollectPose.getHeading(), ScorePose.getHeading())
                .setVelocityConstraint(0.6)
                .build();

        // ========= ROW 2 =========
        rowHPickup = follower.pathBuilder()
                .addPath(new BezierLine(ScorePose, RHuman))
                .setConstantHeadingInterpolation(RHuman.getHeading())
                .setVelocityConstraint(0.1)
                .build();

        rowHToScore = follower.pathBuilder()
                .addPath(new BezierLine(RHuman, ScorePose))
                .setLinearHeadingInterpolation(RHuman.getHeading(), ScorePose.getHeading())
                .setVelocityConstraint(0.6)
                .build();
        scoreToPark = follower.pathBuilder()
                .addPath(new BezierLine(ScorePose, startPose))
                .setLinearHeadingInterpolation(ScorePose.getHeading(), startPose.getHeading())
                .setVelocityConstraint(0.6)
                .build();
    }



    private void updateAutonomous() {
        switch (currentState) {
            // START
            case START:
                outtake.spinUp(scoreShooterTPS);
                RobotHardware.outtakeAngleAdjust.setPosition(MConstants.flapDown);

                follower.followPath(startToScore);
                follower.setMaxPower(0.65);
                transitionTo(AutoState.GO_SCORE_PRELOAD);
                break;

            // SCORE PRELOAD
            case GO_SCORE_PRELOAD:
                if (!follower.isBusy()) {
                    blocker.in();
//                    fireTask = new AutoFireTaskA(outtake, indexer, ejector, intake, scoreShooterTPSArrayFirst);
                    fireTask = new AutoFireTask(outtake, indexer, ejector, intake, ballSensors, scoreShooterTPS);
                    fireTask.start();
                    transitionTo(AutoState.SCORE_PRELOAD);
                }
                break;

            case SCORE_PRELOAD:
                fireTask.update();
                if (fireTask.isActive()) {
                } else {
                    follower.setMaxPower(0.65);
                    intake.runIn();
                    follower.followPath(row3Pickup);
                    transitionTo(AutoState.PICKUP_R3);
                    fireTask = null;
                    blocker.out();
                }
                break;

            // SCORE R1
            case PICKUP_R3:
                if (!follower.isBusy()) {
                    intake.runInAt(0.8);
                    follower.followPath(row3ToScore, 5, true);
                    transitionTo(AutoState.GO_SCORE_R3);
                }
                break;
            case GO_SCORE_R3:
                if (!follower.isBusy()) {
                    blocker.in();
                    turret.runToTick(turret.radToTick(Math.toRadians(7.0))); // opposite. more (-) is right
//                    fireTask = new AutoFireTaskA(outtake, indexer, ejector, intake, scoreShooterTPSArray);
                    fireTask = new AutoFireTask(outtake, indexer, ejector, intake, ballSensors, scoreShooterTPS, 4);
                    fireTask.start();
                    transitionTo(AutoState.SCORE_R3);
                }
                break;
            case SCORE_R3:
                fireTask.update();
                if (fireTask.isActive()) {
                } else {
                    intake.runIn();
                    follower.followPath(rowHPickup);
                    transitionTo(AutoState.PICKUP_RHuman);
                    fireTask = null;
                    blocker.out();
                }
                break;

            // SCORE R2
            case PICKUP_RHuman:
                if (!follower.isBusy()) {
                    intake.runInAt(1.0);
                    follower.followPath(rowHToScore);
                    transitionTo(AutoState.GO_SCORE_RHuman);
                }
                break;
            case GO_SCORE_RHuman:
                if (!follower.isBusy()) {
                    blocker.in();
                    turret.runToTick(turret.radToTick(Math.toRadians(6.5)));
                    //                    fireTask = new AutoFireTaskA(outtake, indexer, ejector, intake, scoreShooterTPSArray);
                    fireTask = new AutoFireTask(outtake, indexer, ejector, intake, ballSensors, scoreShooterTPS);
                    fireTask.start();
                    transitionTo(AutoState.SCORE_RHuman);
                }
                break;
            case SCORE_RHuman:
                fireTask.update();
                if (fireTask.isActive()) {
                } else {
                    intake.runIn();
                    follower.followPath(scoreToPark);
                    transitionTo(AutoState.EXIT);
                    fireTask = null;
                    blocker.out();
                }
                break;

            // PARK
            case EXIT:
                if (!follower.isBusy()) {
                    outtake.stop();
                    indexer.stop();
                    intake.stop();
                    ejector.down();
                    blocker.out();
                    transitionTo(AutoState.DONE);
                }
                break;

            case DONE:
                break;
        }
    }

    private void transitionTo(AutoState next) {
        currentState = next;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        RobotHardware.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        intake = new IntakeAction(hardwareMap);
        indexer = new IndexAction(hardwareMap);
        ejector = new EjectorAction(hardwareMap);
        outtake = new OuttakeAction(hardwareMap);
        blocker = new BlockerAction(hardwareMap);
        turret = new TurretAction(hardwareMap);

        ballSensors = new BallSensorArray();

        turret.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // reset to 0
        turret.runToTick((int) turret.radToTick(Math.toRadians(-3))); // left 4 degrees

        ejector.up();
        ejector.down();

        blocker.out();

        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        currentState = AutoState.START;
    }

    @Override
    public void loop() {
        follower.update();

        outtake.spinUp(scoreShooterTPS);
        outtake.update();

        updateAutonomous();

        telemetry.addLine("=== AutonR Debug ===");
        telemetry.addData("State", currentState);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Timer (ms)", pathTimer.getElapsedTime());
        telemetry.addLine("Pose:");
        telemetry.addData("x", "%.2f", follower.getPose().getX());
        telemetry.addData("y", "%.2f", follower.getPose().getY());
        telemetry.addData("heading", "%.2f°", Math.toDegrees(follower.getPose().getHeading()));

        // show shooter velocity if available
        try {
            double shooterVel = RobotHardware.outtakeMotor.getVelocity();
            telemetry.addData("shooter TPS", "%.1f", shooterVel);
            telemetry.addData("shooter goal", scoreShooterTPS);
            telemetry.addData("atTarget", outtake.isAtTargetVelocity());
//            telemetry.addData("fire count", fireCount);
            telemetry.addData("eject angle", ejector.paddle.getPosition());
        } catch (Exception e) {
            telemetry.addLine("Outtake motor velocity unavailable (check RobotHardware field).");
        }

        telemetry.update();
    }
}