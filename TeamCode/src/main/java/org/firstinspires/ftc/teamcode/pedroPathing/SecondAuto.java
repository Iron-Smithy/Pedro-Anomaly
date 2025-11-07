package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.tools.MotorGroup;
import static org.firstinspires.ftc.teamcode.RobotHardware.*;

/**
 * FTC Autonomous using PedroPathing
 * Moves to scoring position, scores, collects balls    , and re-scores.
 */
@Autonomous(name = "secondTry", group = "AAA")
public class SecondAuto extends OpMode {

    // ========================= Hardware =========================
    private MotorGroup outtakeMotors = new MotorGroup(outtakeMotor1, outtakeMotor2);

    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    private Actions actions;

    // ========================= Path Definitions =========================
    private final Pose startPose = new Pose(120, 120, Math.toRadians(270));
    private final Pose scorePose = new Pose(84, 84, Math.toRadians(45));
    private final Pose R1 = new Pose(120, 83.5, Math.toRadians(0));

    private Path scorePreload, eatR1, scoreR1;

    // ========================= State Management =========================
    private enum AutoState {
        START,
        SCORE_PRELOAD,
        INTAKE_R1,
        SCORE_R1,
        DONE
    }

    private AutoState currentState = AutoState.START;

    // ========================= Path Building =========================
    private void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        eatR1 = new Path(new BezierLine(scorePose, R1))
            .setLinearHeadingInterpolation(
                scorePose.getHeading(),
                Math.toRadians(0),
                0.4 // finish turn 40% into path, not at end
            );
         // .setConstantHeadingInterpolation(R1.getHeading()); // face intake direction entire path
         // .setLinearHeadingInterpolation(scorePose.getHeading(), R1.getHeading()); // normal. turns to late to intake correctly

        scoreR1 = new Path(new BezierLine(R1, scorePose))
                .setLinearHeadingInterpolation(R1.getHeading(), scorePose.getHeading());
    }

    // ========================= Main Auto Logic =========================
    private void updateAutonomous() {
        switch (currentState) {
            case START:
                follower.followPath(scorePreload);
                transitionTo(AutoState.SCORE_PRELOAD);
                break;

            case SCORE_PRELOAD:
                if (!follower.isBusy()) {
                    if (actions.shootAuto(1.0, 1500, 3000)) { // spin-up 1500ms, index 3000ms
                        transitionTo(AutoState.INTAKE_R1);
                    }
                }
                break;

            case INTAKE_R1:
                actions.intakeOn(1);
                if (!follower.isBusy()) {
                    actions.intakeOff();
                    follower.followPath(scoreR1);
                    transitionTo(AutoState.SCORE_R1);
                }
                break;

            case SCORE_R1:
                if (!follower.isBusy()) {
                    if (actions.shootAuto(1.0, 1500, 3000)) {
                        transitionTo(AutoState.DONE);
                    }
                }
                break;

            case DONE:
                actions.stopShooter();
                actions.intakeOff();
                break;
        }
    }


    // ========================= Helper Functions =========================
    private void transitionTo(AutoState next) {
        currentState = next;
        pathTimer.resetTimer();
    }

    private void transitionTo(AutoState next, long delayMs) {
        currentState = next;
        pathTimer.resetTimer();
        pathTimer.setDelay(delayMs); // Optional.
    }

    // ========================= OpMode Lifecycle =========================
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        actions = new Actions();

        telemetry.addLine("FirstAuto initialized.");
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
        currentState = AutoState.START;
    }

    @Override
    public void loop() {
        follower.update();
        updateAutonomous();

        telemetry.addLine("=== Auto Debug ===");
        telemetry.addData("State", currentState);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Timer (ms)", pathTimer.getElapsedTime());
        telemetry.addLine("Pose:");
        telemetry.addData("x", "%.2f", follower.getPose().getX());
        telemetry.addData("y", "%.2f", follower.getPose().getY());
        telemetry.addData("heading", "%.2f°", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        outtakeMotors.setPower(0);
        indexingWheel.setPower(0);
        intakeMotor.setPower(0);
    }
}
