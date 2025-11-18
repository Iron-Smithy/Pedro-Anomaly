package org.firstinspires.ftc.teamcode.pedroPathing;

import android.graphics.Point;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.tools.Actions;
import org.firstinspires.ftc.teamcode.tools.MotorGroup;

@Autonomous(name = "AutonR", group = "AAA")
public class AutonR extends OpMode {

    // ========================= Hardware =========================
    private MotorGroup outtakeMotors;

    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    private Actions actions;

    private long scoreShooterTPS = 0;
    private long toleracnce = 50;

    // ========================= Path Definitions =========================
    private final Pose startPose = new Pose(120, 127, Math.toRadians(-143));
    private final Pose scorePose = new Pose(100, 107, Math.toRadians(-144));
    private final Pose row1 = new Pose(130, 83.5, Math.toRadians(0));
    private final Pose row1CP = new Pose(85, 80, Math.toRadians(0));
    private final Pose row2 = new Pose(120, 59.5, Math.toRadians(0));
    private final Pose row3 = new Pose(120, 35.5, Math.toRadians(0));

    private Path scorePreload, pickUpR1, scoreR1, pickUpR2, scoreR2, pickUpR3, scoreR3;

    // ========================= State Management =========================
    private enum AutoState {
        START,
        GO_SCORE_PRELOAD,
        SCORE_PRELOAD,
        PICKUP_R1,
        GO_SCORE_R1,
        SCORE_R1,
        PICKUP_R2,
        SCORE_R2,
        PICKUP_R3,
        SCORE_R3,
        DONE
    }

    private AutoState currentState = AutoState.START;

    // ========================= Path Building =========================
    private void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        scorePreload.setVelocityConstraint(0.85);

        // row 1
        pickUpR1 = new Path(new BezierCurve(scorePose, row1CP, row1));
        pickUpR1.setConstantHeadingInterpolation(row1.getHeading());
        pickUpR1.setVelocityConstraint(0.1);

        scoreR1 = new Path(new BezierLine(row1, scorePose));
        scoreR1.setLinearHeadingInterpolation(row1.getHeading(), scorePose.getHeading());

        // row 2
        pickUpR2 = new Path(new BezierLine(scorePose, row2));
        pickUpR2.setConstantHeadingInterpolation(row2.getHeading());
        pickUpR2.setVelocityConstraint(0.1);

        scoreR2 = new Path(new BezierLine(row2, scorePose));
        scoreR2.setLinearHeadingInterpolation(row2.getHeading(), scorePose.getHeading());

        // row 3
        pickUpR3 = new Path(new BezierLine(scorePose, row3));
        pickUpR3.setConstantHeadingInterpolation(row3.getHeading());
        pickUpR3.setVelocityConstraint(0.1);

        scoreR3 = new Path(new BezierLine(row3, scorePose));
        scoreR3.setLinearHeadingInterpolation(row3.getHeading(), scorePose.getHeading());

//        score = new BezierCurve(follower::getPose, scorePose);
    }

    // ========================= Main Auto Logic =========================
    private void updateAutonomous() {
        switch (currentState) {
            case START:
                actions.spinUpShooter(2120, 1.0);
                actions.intakeOn(-1);

                follower.followPath(scorePreload);
                transitionTo(AutoState.GO_SCORE_PRELOAD);
                break;

            case GO_SCORE_PRELOAD:
                if ((!follower.isBusy() && outtakeMotors.metVelocity(scoreShooterTPS, toleracnce))
                        || pathTimer.getElapsedTime() >= 5000) {
                    transitionTo(AutoState.SCORE_PRELOAD);
                }
                break;
            case SCORE_PRELOAD:
                if (outtakeMotors.metVelocity(scoreShooterTPS, toleracnce)) {
                    actions.indexOn(1);
                } else {
                    actions.indexOff();
                }
                if (pathTimer.getElapsedTime() >= 4000) {
                    actions.indexOff();
                    actions.stopShooter();

                    follower.followPath(pickUpR1);
                    transitionTo(AutoState.PICKUP_R1);
                }
                break;

            case PICKUP_R1:
                if ((!follower.isBusy() && pathTimer.getElapsedTime() >= 2000) || pathTimer.getElapsedTime() >= 7000) {
                    follower.followPath(scoreR1);
                    transitionTo(AutoState.SCORE_R1);

                    actions.outtakeSpinUp(scoreShooterTPS, 1);
                }
                break;
            case GO_SCORE_R1:
                if ((!follower.isBusy() && outtakeMotors.metVelocity(scoreShooterTPS, toleracnce))
                        || pathTimer.getElapsedTime() >= 4000) {
                    transitionTo(AutoState.DONE);
                }

                break;
            case SCORE_R1:
                if (outtakeMotors.metVelocity(scoreShooterTPS, toleracnce)) {
                    actions.indexOn(1);
                } else {
                    actions.indexOff();
                }
                if (pathTimer.getElapsedTime() >= 3000) {
                    actions.indexOff();
                    actions.stopShooter();

                    transitionTo(AutoState.DONE);
                }
                break;

            case DONE:
                actions.stopShooter();
                actions.intakeOff();
                actions.indexOff();
                break;
        }
    }


    // ========================= Helper Functions =========================
    private void transitionTo(AutoState next) {
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
        telemetry.addData("motor TPS:", outtakeMotors.getVelocity());
        telemetry.addData("motor TPS goal:", scoreShooterTPS);
        telemetry.update();
    }

    @Override
    public void stop() {
        actions.intakeOff();
        actions.stopShooter();
    }
}
