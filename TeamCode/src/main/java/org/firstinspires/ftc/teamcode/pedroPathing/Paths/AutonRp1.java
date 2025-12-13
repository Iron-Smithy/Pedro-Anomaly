package org.firstinspires.ftc.teamcode.pedroPathing.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.tools.Actions.AutoDriveTask;
import org.firstinspires.ftc.teamcode.tools.Actions.AutoFireTask;
import org.firstinspires.ftc.teamcode.tools.Actions.EjectorAction;
import org.firstinspires.ftc.teamcode.tools.Actions.IndexAction;
import org.firstinspires.ftc.teamcode.tools.Actions.IntakeAction;
import org.firstinspires.ftc.teamcode.tools.Actions.OuttakeAction;
import org.firstinspires.ftc.teamcode.tools.ButtonHandler;

@Autonomous(name = "AutonRp1", group = "AAA")
public class AutonRp1 extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    private IntakeAction intake;
    private IndexAction indexer;
    private EjectorAction ejector;
    private OuttakeAction outtake;

    private long scoreShooterTPS = 1100;
    private long tolerance = 50;

    private AutoFireTask fireTask = null;

    private final Pose startPose = new Pose(/*144-*/120, 127, Math.toRadians(/*180-*/37));
    private final Pose Score = new Pose(/*144-*/100, 107, Math.toRadians(/*180-*/47));
    private final Pose R1Pre = new Pose(/*144-*/100, 82.5, Math.toRadians(/*180-*/0));
    private final Pose R1Collect = new Pose(/*144-*/125, 82.5, Math.toRadians(/*180-*/0));
    private final Pose ScoreR1CP = new Pose(/*144-*/100, 82.5, Math.toRadians(/*180-*/0));
    private final Pose R2Pre = new Pose(/*144-*/100, 60, Math.toRadians(/*180-*/0));
    private final Pose R2Collect = new Pose(/*144-*/125, 60, Math.toRadians(/*180-*/0));
    private final Pose ScoreR2CP = new Pose(/*144-*/90, 55, Math.toRadians(/*180-*/0));
    private final Pose R3Pre = new Pose(/*144-*/100, 35, Math.toRadians(/*180-*/0));
    private final Pose R3Collect = new Pose(/*144-*/125, 35, Math.toRadians(/*180-*/0));
    private final Pose ScoreR3CP = new Pose(/*144-*/90, 30, Math.toRadians(/*180-*/0));
    private final Pose Park = new Pose(/*144-*/120, 127, Math.toRadians(/*180-*/37)); // start pose, make right

    private enum AutoState {
        START,
        GO_SCORE_PRELOAD,
        SCORE_PRELOAD,

        PICKUP_R1,
        GO_SCORE_R1,
        SCORE_R1,

        PICKUP_R2,
        GO_SCORE_R2,
        SCORE_R2,

        PICKUP_R3,
        GO_SCORE_R3,
        SCORE_R3,

        EXIT,
        DONE
    }

    private AutoState currentState = AutoState.START;

    private PathChain startToScore;

    private PathChain row1Pickup;
    private PathChain row1Return;

    private PathChain row2Pickup;
    private PathChain row2Return;

    private PathChain row3Pickup;
    private PathChain row3Return;

    private PathChain park;

    private void buildPaths() {
        // ========= START → SCORE =========
        startToScore = follower.pathBuilder()
                .addPath(new BezierLine(startPose, Score))
                .setLinearHeadingInterpolation(startPose.getHeading(), Score.getHeading())
                .build();

        // ========= ROW 1 =========
        row1Pickup = follower.pathBuilder()
                .addPath(new BezierLine(Score, R1Pre))
                .setLinearHeadingInterpolation(Score.getHeading(), R1Pre.getHeading())
                .addPath(new BezierLine(R1Pre, R1Collect))
                .setLinearHeadingInterpolation(R1Pre.getHeading(), R1Collect.getHeading())
                .build();

        row1Return = follower.pathBuilder()
                // You can still use control points by using the addPath method that takes multiple Poses
                .addPath(new BezierCurve(R1Collect, ScoreR1CP, Score))
                .setLinearHeadingInterpolation(R1Collect.getHeading(), Score.getHeading())
                .build();

        // ========= ROW 2 =========
        row2Pickup = follower.pathBuilder()
                .addPath(new BezierLine(Score, R2Pre))
                .setLinearHeadingInterpolation(Score.getHeading(), R2Pre.getHeading())
                .addPath(new BezierLine(R2Pre, R2Collect))
                .setLinearHeadingInterpolation(R2Pre.getHeading(), R2Collect.getHeading())
                .build();

        row2Return = follower.pathBuilder()
                .addPath(new BezierCurve(R2Collect, ScoreR2CP, Score))
                .setLinearHeadingInterpolation(R2Collect.getHeading(), Score.getHeading())
                .build();

        // ========= ROW 3 =========
        row3Pickup = follower.pathBuilder()
                .addPath(new BezierLine(Score, R3Pre))
                .setLinearHeadingInterpolation(Score.getHeading(), R3Pre.getHeading())
                .addPath(new BezierLine(R3Pre, R3Collect))
                .setLinearHeadingInterpolation(R3Pre.getHeading(), R3Collect.getHeading())
                .build();

        row3Return = follower.pathBuilder()
                .addPath(new BezierCurve(R3Collect, ScoreR3CP, Score))
                .setLinearHeadingInterpolation(R3Collect.getHeading(), Score.getHeading())
                .build();

        // ========= Park =========
        park = follower.pathBuilder()
                .addPath(new BezierLine(Score, startPose))
                .setLinearHeadingInterpolation(Score.getHeading(), startPose.getHeading())
                .build();
    }



    private void updateAutonomous() {
        switch (currentState) {
            // START
            case START:
                outtake.spinUp(scoreShooterTPS);
                intake.runIn();
                RobotHardware.outtakeAngleAdjust.setPosition(0.3);

                follower.followPath(startToScore);
                transitionTo(AutoState.GO_SCORE_PRELOAD);
                break;

            // SCORE PRELOAD
            case GO_SCORE_PRELOAD:
                if (!follower.isBusy()) {
                    fireTask = new AutoFireTask(outtake, indexer, ejector, intake, scoreShooterTPS);
                    fireTask.start();
                    transitionTo(AutoState.SCORE_PRELOAD);
                }
                break;
            case SCORE_PRELOAD:
                fireTask.update();
                if (fireTask.isActive()) {
                } else {
                    follower.followPath(row1Pickup);
                    transitionTo(AutoState.PICKUP_R1);
                    fireTask = null;
                }
                break;

            // SCORE R1
            case PICKUP_R1:
                if (!follower.isBusy()) {
                    follower.followPath(row1Return);
                    transitionTo(AutoState.GO_SCORE_R1);
                }
                break;
            case GO_SCORE_R1:
                if (!follower.isBusy()) {
                    fireTask = new AutoFireTask(outtake, indexer, ejector, intake, scoreShooterTPS);
                    fireTask.start();
                    transitionTo(AutoState.SCORE_R1);
                }
                break;
            case SCORE_R1:
                fireTask.update();
                if (fireTask.isActive()) {
                } else {
                    follower.followPath(row2Pickup);
                    transitionTo(AutoState.PICKUP_R2);
                    fireTask = null;
                }
                break;

            // SCORE R2
            case PICKUP_R2:
                if (!follower.isBusy()) {
                    follower.followPath(row2Return);
                    transitionTo(AutoState.GO_SCORE_R2);
                }
                break;
            case GO_SCORE_R2:
                if (!follower.isBusy()) {
                    fireTask = new AutoFireTask(outtake, indexer, ejector, intake, scoreShooterTPS);
                    fireTask.start();
                    transitionTo(AutoState.SCORE_R2);
                }
                break;
            case SCORE_R2:
                fireTask.update();
                if (fireTask.isActive()) {
                } else {
                    follower.followPath(row3Pickup);
                    transitionTo(AutoState.PICKUP_R3);
                    fireTask = null;
                }
                break;

            // SCORE R3
            case PICKUP_R3:
                if (!follower.isBusy()) {
                    follower.followPath(row3Return);
                    transitionTo(AutoState.GO_SCORE_R3);
                }
                break;
            case GO_SCORE_R3:
                if (!follower.isBusy()) {
                    fireTask = new AutoFireTask(outtake, indexer, ejector, intake, scoreShooterTPS);
                    fireTask.start();
                    transitionTo(AutoState.SCORE_R3);
                }
                break;
            case SCORE_R3:
                fireTask.update();
                if (fireTask.isActive()) {
                } else {
                    follower.followPath(park);
                    transitionTo(AutoState.EXIT);
                    fireTask = null;
                }
                break;

            // PARK
            case EXIT:
                if (!follower.isBusy()) {
                    outtake.stop();
                    indexer.stop();
                    intake.stop();
                    ejector.down();
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
        intake.runIn();
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