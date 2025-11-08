package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.RobotHardware.indexingWheel;
import static org.firstinspires.ftc.teamcode.RobotHardware.intakeMotor;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.tools.Actions;
import org.firstinspires.ftc.teamcode.tools.MotorGroup;


/**
 * FTC Autonomous using PedroPathing
 * Moves to scoring position, scores, collects balls    , and re-scores.
 */
@Autonomous(name = "secondTryB", group = "AAA")
public class SecondAutoB extends OpMode {

    // ========================= Hardware =========================
    private MotorGroup outtakeMotors;

    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    private Actions actions;

    // ========================= Path Definitions =========================
    private final Pose startPose = new Pose(24, 120, Math.toRadians(-35));
    private final Pose scorePose = new Pose(60, 84, Math.toRadians(-44));
    private final Pose tR1 = new Pose(54, 80,Math.toRadians(180));
    private final Pose R1 = new Pose(24, 80, Math.toRadians(180));

    private Path scorePreload, scoreR1, turnR1, eatR1;
    private PathChain eatR1PC;

    // ========================= State Management =========================
    private enum AutoState {
        START,
        REV,
        SCORE_PRELOAD,
        TURN_R1,
        INTAKE_R1,
        SCORE_R1,
        DONE
    }

    private AutoState currentState = AutoState.START;

    // ========================= Path Building =========================
    private void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        turnR1 = new Path(new BezierLine(scorePose, tR1));
        turnR1.setLinearHeadingInterpolation(scorePose.getHeading(), tR1.getHeading());

        eatR1 = new Path(new BezierLine(tR1, R1));
        eatR1.setLinearHeadingInterpolation(
                tR1.getHeading(),
                Math.toRadians(180)
        );
         // .setConstantHeadingInterpolation(R1.getHeading()); // face intake direction entire path
         // .setLinearHeadingInterpolation(scorePose.getHeading(), R1.getHeading()); // normal. turns to late to intake correctly

        eatR1PC = new PathChain(turnR1, eatR1);

        scoreR1 = new Path(new BezierLine(R1, scorePose));
        scoreR1.setLinearHeadingInterpolation(R1.getHeading(), scorePose.getHeading());
    }

    // ========================= Main Auto Logic =========================
    private void updateAutonomous() {
        switch (currentState) {
            case START:
                follower.followPath(scorePreload);
                transitionTo(AutoState.REV);
                break;

            case REV:
                actions.spinUpShooter(2120, 1.0);
                if (!follower.isBusy() || pathTimer.getElapsedTime() >= 3000) {
                    transitionTo(AutoState.SCORE_PRELOAD);
                }
                break;

            case SCORE_PRELOAD:
                actions.spinUpShooter(2120, 1.0);
                actions.intakeOn(-1);

                if (actions.shootAuto(2120, 1.0, 1500, 6000, pathTimer.getElapsedTime())) {
                    follower.followPath(turnR1);
                    transitionTo(AutoState.TURN_R1);
                }
                break;

            case TURN_R1:
                actions.spinUpShooter(2120, 1.0);
                actions.intakeOn(-1);

                if (actions.shootAuto(2120, 1.0, 1500, 6000, pathTimer.getElapsedTime())) {
                    follower.followPath(eatR1);
                    transitionTo(AutoState.INTAKE_R1);
                }
                break;

            case INTAKE_R1:
                actions.intakeOn(-1);
                if (!follower.isBusy() || pathTimer.getElapsedTime() >= 3000) {
                    actions.intakeOff();
//                    follower.followPath(scoreR1);
                    transitionTo(AutoState.DONE);
                }
                break;

//            case SCORE_R1:
//                actions.spinUpShooter(2300, 1.0);
//                if (!follower.isBusy() || pathTimer.getElapsedTime() >= 3000) {
//                    actions.intakeOn(-1);
//
//                    if (actions.shootAuto(2300, 1.0, 1500, 2000, pathTimer.getElapsedTime()-3000)) {
//                        transitionTo(AutoState.DONE);
//                    }
//                }
//                break;

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
    }

    // ========================= OpMode Lifecycle =========================
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        RobotHardware.init(hardwareMap);
        outtakeMotors = new MotorGroup(RobotHardware.outtakeMotor1, RobotHardware.outtakeMotor2);
        outtakeMotors.setUsingEncoder();

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
