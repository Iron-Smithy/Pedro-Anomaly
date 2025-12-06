package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.tools.Actions.EjectorAction;
import org.firstinspires.ftc.teamcode.tools.Actions.IntakeAction;
import org.firstinspires.ftc.teamcode.tools.Actions.IndexAction;
import org.firstinspires.ftc.teamcode.tools.Actions.OuttakeAction;
import org.firstinspires.ftc.teamcode.pedroPathing.MConstants;

@Autonomous(name = "AutonRE", group = "AAA")
public class AutonRE extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer, RunTimer;

    private IntakeAction intake;
    private IndexAction indexer;
    private EjectorAction ejector;
    private OuttakeAction outtake;

    private long scoreShooterTPS = 1050;
    private long tolerance = 50;

    private long fireCount = 0;

    private final Pose startPose = new Pose(120, 127, Math.toRadians(37));
    private final Pose scorePose = new Pose(100, 107, Math.toRadians(36));

    private final Pose row1 = new Pose(130, 83.5, Math.toRadians(0));
    private final Pose row1CP = new Pose(85, 80, Math.toRadians(0));
    private final Pose row2 = new Pose(130, 59.5, Math.toRadians(0));
    private final Pose row2CP = new Pose(71, 53, Math.toRadians(0));

    private Path scorePreload, pickUpR1, scoreR1, pickUpR2, scoreR2;

    private enum AutoState {
        START,
        GO_SCORE_PRELOAD,
        SCORE_PRELOAD,
        SCORE_PRELOAD_SERVO,
        PICKUP_R1,
        GO_SCORE_R1,
        SCORE_R1,
        SCORE_R1_SERVO,
        PICKUP_R2,
        GO_SCORE_R2,
        SCORE_R2,
        SCORE_R2_SERVO,
        DONE
    }

    private AutoState currentState = AutoState.START;

    private void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        scorePreload.setVelocityConstraint(0.8);

        pickUpR1 = new Path(new BezierCurve(scorePose, row1CP, row1));
        pickUpR1.setConstantHeadingInterpolation(row1.getHeading());
        pickUpR1.setVelocityConstraint(0.2);

        scoreR1 = new Path(new BezierLine(row1, scorePose));
        scoreR1.setLinearHeadingInterpolation(row1.getHeading(), scorePose.getHeading());

        pickUpR2 = new Path(new BezierCurve(scorePose, row2CP, row2));
        pickUpR2.setConstantHeadingInterpolation(row2.getHeading());
        pickUpR2.setVelocityConstraint(0.2);

        scoreR2 = new Path(new BezierCurve(row2, row2CP, scorePose));
        scoreR2.setLinearHeadingInterpolation(row2.getHeading(), scorePose.getHeading());
    }

    private void updateAutonomous() {
        switch (currentState) {
            case START:
                outtake.spinUp(scoreShooterTPS);
                intake.runIn();
                RobotHardware.outtakeAngleAdjust.setPosition(0.3);
                follower.followPath(scorePreload);
                transitionTo(AutoState.GO_SCORE_PRELOAD);
                break;

            case GO_SCORE_PRELOAD:
                if ((!follower.isBusy() && outtake.isAtTargetVelocity()) || pathTimer.getElapsedTime() >= 5000) {
                    transitionTo(AutoState.SCORE_PRELOAD);
                }
                break;
            case SCORE_PRELOAD:
                if (outtake.isAtTargetVelocity()) indexer.runIn();
                ejector.down();
                if (fireCount < 3) {
                    if (pathTimer.getElapsedTime() >= 750 && outtake.isAtTargetVelocity()) { // small delay between shots
                        ejector.up();
                        transitionTo(AutoState.SCORE_PRELOAD_SERVO);
                    }
                } else {
                    fireCount = 0;
                    indexer.stop();
                    follower.followPath(pickUpR1);
                    transitionTo(AutoState.PICKUP_R1);
                }
                break;
            case SCORE_PRELOAD_SERVO:
                if (pathTimer.getElapsedTime() >= 300) {
                    fireCount++;
                    transitionTo(AutoState.SCORE_PRELOAD);
                }
                break;

            case PICKUP_R1:
                if (!follower.isBusy() || pathTimer.getElapsedTime() >= 6000) {
                    follower.followPath(scoreR1);
                    transitionTo(AutoState.GO_SCORE_R1);
                }
                break;
            case GO_SCORE_R1:
                if (!follower.isBusy() || pathTimer.getElapsedTime() >= 6000) {
                    transitionTo(AutoState.SCORE_R1);
                }
                break;

            case SCORE_R1:
                if (outtake.isAtTargetVelocity()) indexer.runIn();
                ejector.down();
                if (fireCount < 3) {
                    if (pathTimer.getElapsedTime() >= 750 && outtake.isAtTargetVelocity()) { // small delay between shots
                        ejector.up();
                        transitionTo(AutoState.SCORE_R1_SERVO);
                    }
                } else {
                    fireCount = 0;
                    indexer.stop();
                    follower.followPath(pickUpR2);
                    transitionTo(AutoState.PICKUP_R2);
                }
                break;
            case SCORE_R1_SERVO:
                if (pathTimer.getElapsedTime() >= 300) {
                    fireCount++;
                    transitionTo(AutoState.SCORE_R1);
                }
                break;

            case PICKUP_R2:
                if (!follower.isBusy() || pathTimer.getElapsedTime() >= 6000) {
                    follower.followPath(scoreR2);
                    transitionTo(AutoState.GO_SCORE_R2);
                }
                break;
            case GO_SCORE_R2:
                if (!follower.isBusy() || pathTimer.getElapsedTime() >= 4000) {
                    transitionTo(AutoState.SCORE_R2);
                }
                break;

            case SCORE_R2:
                if (outtake.isAtTargetVelocity()) indexer.runIn();
                ejector.down();
                if (fireCount < 3) {
                    if (pathTimer.getElapsedTime() >= 750 && outtake.isAtTargetVelocity()) { // small delay between shots
                        ejector.up();
                        transitionTo(AutoState.SCORE_R2_SERVO);
                    }
                } else {
                    fireCount = 0;
                    indexer.stop();
                    transitionTo(AutoState.DONE);
                }
                break;
            case SCORE_R2_SERVO:
                if (pathTimer.getElapsedTime() >= 300) {
                    fireCount++;
                    transitionTo(AutoState.SCORE_R2);
                }
                break;

            case DONE:
                outtake.stop();
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
            telemetry.addData("fire count", fireCount);
            telemetry.addData("eject angle", ejector.paddle.getPosition());
        } catch (Exception e) {
            telemetry.addLine("Outtake motor velocity unavailable (check RobotHardware field).");
        }

        telemetry.update();
    }
}
