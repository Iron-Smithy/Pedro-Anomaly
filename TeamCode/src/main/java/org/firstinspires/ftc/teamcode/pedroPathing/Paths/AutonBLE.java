//package org.firstinspires.ftc.teamcode.pedroPathing;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.Path;
//import com.pedropathing.paths.PathChain;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.teamcode.RobotHardware;
//import org.firstinspires.ftc.teamcode.tools.Actions.EjectorAction;
//import org.firstinspires.ftc.teamcode.tools.Actions.IndexAction;
//import org.firstinspires.ftc.teamcode.tools.Actions.IntakeAction;
//import org.firstinspires.ftc.teamcode.tools.Actions.OuttakeAction;
//
//@Autonomous(name = "AutonBLE", group = "ZZZ")
//public class AutonBLE extends OpMode {
//    private Follower follower;
//    private Timer pathTimer, opmodeTimer;
//
//    private IntakeAction intake;
//    private IndexAction indexer;
//    private EjectorAction ejector;
//    private OuttakeAction outtake;
//
//    private long scoreShooterTPS = 1150;
//    private long tolerance = 50;
//
//    private long fireCount = 0;
//
//    private final Pose startPose = new Pose(144-120, 127, Math.toRadians(180-37));
//    private final Pose Score = new Pose(144-100, 107, Math.toRadians(180-47));
//    private final Pose R1Pre = new Pose(144-100, 82.5, Math.toRadians(180-0));
//    private final Pose R1Collect = new Pose(144-125, 82.5, Math.toRadians(180-0));
//    private final Pose ScoreR1CP = new Pose(144-100, 82.5, Math.toRadians(180));
//    private final Pose R2Pre = new Pose(144-100, 60, Math.toRadians(180-0));
//    private final Pose R2Collect = new Pose(144-125, 60, Math.toRadians(180-0));
//    private final Pose ScoreR2CP = new Pose(144-90, 55, Math.toRadians(180));
//    private final Pose R3Pre = new Pose(144-100, 35, Math.toRadians(180-0));
//    private final Pose R3Collect = new Pose(144-125, 35, Math.toRadians(180-0));
//    private final Pose ScoreR3CP = new Pose(144-90, 30, Math.toRadians(180-0));
//
//
//    private Path scorePreload, pickUpR1Pre, pickUpR1, scoreR1, pickUpR2Pre, pickUpR2, scoreR2, pickUpR3Pre, pickUpR3, scoreR3;
//    private PathChain collectR1, collectR2, collectR3;
//
//    private enum AutoState {
//        START,
//        GO_SCORE_PRELOAD,
//        SCORE_PRELOAD,
//        SCORE_PRELOAD_SERVO,
//        PICKUP_R1,
//        GO_SCORE_R1,
//        SCORE_R1,
//        SCORE_R1_SERVO,
//        PICKUP_R2,
//        GO_SCORE_R2,
//        SCORE_R2,
//        SCORE_R2_SERVO,
//        PICKUP_R3,
//        GO_SCORE_R3,
//        SCORE_R3,
//        SCORE_R3_SERVO,
//        DONE
//    }
//
//    private AutoState currentState = AutoState.START;
//
//    private void buildPaths() {
//        scorePreload = new Path(new BezierLine(startPose, Score));
//        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), Score.getHeading());
//        scorePreload.setVelocityConstraint(0.8);
//
//        pickUpR1Pre = new Path(new BezierLine(Score, R1Pre));
//        pickUpR1Pre.setConstantHeadingInterpolation(R1Pre.getHeading());
//        pickUpR1Pre.setVelocityConstraint(0.2);
//
//        pickUpR1 = new Path(new BezierLine(R1Pre, R1Collect));
//        pickUpR1.setConstantHeadingInterpolation(R1Collect.getHeading());
//        pickUpR1.setVelocityConstraint(0.2);
//
//        scoreR1 = new Path(new BezierCurve(R1Collect, ScoreR1CP, Score));
//        scoreR1.setLinearHeadingInterpolation(R1Collect.getHeading(), Score.getHeading());
//
//        pickUpR2Pre = new Path(new BezierLine(Score, R2Pre));
//        pickUpR2Pre.setConstantHeadingInterpolation(R2Pre.getHeading());
//        pickUpR2Pre.setVelocityConstraint(0.2);
//
//        pickUpR2 = new Path(new BezierLine(R2Pre, R2Collect));
//        pickUpR2.setConstantHeadingInterpolation(R2Collect.getHeading());
//        pickUpR2.setVelocityConstraint(0.2);
//
//        scoreR2 = new Path(new BezierCurve(R2Collect, ScoreR2CP, Score));
//        scoreR2.setLinearHeadingInterpolation(R2Collect.getHeading(), Score.getHeading());
//
//        pickUpR3Pre = new Path(new BezierLine(Score, R3Pre));
//        pickUpR3Pre.setConstantHeadingInterpolation(R3Pre.getHeading());
//        pickUpR3Pre.setVelocityConstraint(0.2);
//
//        pickUpR3 = new Path(new BezierLine(R3Pre, R3Collect));
//        pickUpR3.setConstantHeadingInterpolation(R3Collect.getHeading());
//        pickUpR3.setVelocityConstraint(0.2);
//
//        scoreR3 = new Path(new BezierCurve(R3Collect, ScoreR3CP, Score));
//        scoreR3.setLinearHeadingInterpolation(R3Collect.getHeading(), Score.getHeading());
//
//        collectR1 = follower.pathBuilder()
//                .addPath(new BezierLine(startPose, pickUpR1))
//                .setConstantHeadingInterpolation(R1Collect.getHeading())
//                .build();
//    }
//
//    private void updateAutonomous() {
//        switch (currentState) {
//            case START:
//                outtake.spinUp(scoreShooterTPS);
//                intake.runIn();
//                RobotHardware.outtakeAngleAdjust.setPosition(0.3);
//                follower.followPath(scorePreload);
//                transitionTo(AutoState.GO_SCORE_PRELOAD);
//                break;
//
//            case GO_SCORE_PRELOAD:
//                if ((!follower.isBusy() && outtake.isAtTargetVelocity()) || pathTimer.getElapsedTime() >= 8000) {
//                    transitionTo(AutoState.SCORE_PRELOAD);
//                }
//                break;
//            case SCORE_PRELOAD:
//                if (outtake.isAtTargetVelocity()) indexer.runIn();
//                ejector.down();
//                if (fireCount < 3) {
//                    if (pathTimer.getElapsedTime() >= 500 && outtake.isAtTargetVelocity()) { // small delay between shots
//                        ejector.up();
//                        transitionTo(AutoState.SCORE_PRELOAD_SERVO);
//                    }
//                } else {
//                    fireCount = 0;
//                    indexer.stop();
//                    follower.followPath(pickUpR1);
//                    transitionTo(AutoState.PICKUP_R1);
//                }
//                break;
//            case SCORE_PRELOAD_SERVO:
//                if (pathTimer.getElapsedTime() >= 300) {
//                    fireCount++;
//                    transitionTo(AutoState.SCORE_PRELOAD);
//                }
//                break;
//
//            case PICKUP_R1:
//                if (!follower.isBusy() || pathTimer.getElapsedTime() >= 8000) {
//                    follower.followPath(scoreR1);
//                    transitionTo(AutoState.GO_SCORE_R1);
//                }
//                break;
//            case GO_SCORE_R1:
//                if (!follower.isBusy() || pathTimer.getElapsedTime() >= 8000) {
//                    transitionTo(AutoState.SCORE_R1);
//                }
//                break;
//
//            case SCORE_R1:
//                if (outtake.isAtTargetVelocity()) indexer.runIn();
//                ejector.down();
//                if (fireCount < 3) {
//                    if (pathTimer.getElapsedTime() >= 500 && outtake.isAtTargetVelocity()) { // small delay between shots
//                        ejector.up();
//                        transitionTo(AutoState.SCORE_R1_SERVO);
//                    }
//                } else {
//                    fireCount = 0;
//                    indexer.stop();
//                    follower.followPath(pickUpR2);
//                    transitionTo(AutoState.PICKUP_R2);
//                }
//                break;
//            case SCORE_R1_SERVO:
//                if (pathTimer.getElapsedTime() >= 300) {
//                    fireCount++;
//                    transitionTo(AutoState.SCORE_R1);
//                }
//                break;
//
//            case PICKUP_R2:
//                if (!follower.isBusy() || pathTimer.getElapsedTime() >= 8000) {
//                    follower.followPath(scoreR2);
//                    transitionTo(AutoState.GO_SCORE_R2);
//                }
//                break;
//            case GO_SCORE_R2:
//                if (!follower.isBusy() || pathTimer.getElapsedTime() >= 8000) {
//                    transitionTo(AutoState.SCORE_R2);
//                }
//                break;
//
//            case SCORE_R2:
//                if (outtake.isAtTargetVelocity()) indexer.runIn();
//                ejector.down();
//                if (fireCount < 3) {
//                    if (pathTimer.getElapsedTime() >= 500 && outtake.isAtTargetVelocity()) { // small delay between shots
//                        ejector.up();
//                        transitionTo(AutoState.SCORE_R2_SERVO);
//                    }
//                } else {
//                    fireCount = 0;
//                    indexer.stop();
//                    follower.followPath(pickUpR3);
//                    transitionTo(AutoState.PICKUP_R3);
//                }
//                break;
//            case SCORE_R2_SERVO:
//                if (pathTimer.getElapsedTime() >= 300) {
//                    fireCount++;
//                    transitionTo(AutoState.SCORE_R2);
//                }
//                break;
//            case PICKUP_R3:
//                if (!follower.isBusy() || pathTimer.getElapsedTime() >= 8000) {
//                    follower.followPath(scoreR3);
//                    transitionTo(AutoState.GO_SCORE_R3);
//                }
//                break;
//            case GO_SCORE_R3:
//                if (!follower.isBusy() || pathTimer.getElapsedTime() >= 8000) {
//                    transitionTo(AutoState.SCORE_R3);
//                }
//                break;
//
//            case SCORE_R3:
//                if (outtake.isAtTargetVelocity()) indexer.runIn();
//                ejector.down();
//                if (fireCount < 3) {
//                    if (pathTimer.getElapsedTime() >= 500 && outtake.isAtTargetVelocity()) { // small delay between shots
//                        ejector.up();
//                        transitionTo(AutoState.SCORE_R3_SERVO);
//                    }
//                } else {
//                    fireCount = 0;
//                    indexer.stop();
//                    transitionTo(AutoState.DONE);
//                }
//                break;
//            case SCORE_R3_SERVO:
//                if (pathTimer.getElapsedTime() >= 300) {
//                    fireCount++;
//                    transitionTo(AutoState.SCORE_R3);
//                }
//                break;
//
//
//            case DONE:
//                outtake.stop();
//                indexer.stop();
//                intake.stop();
//                ejector.down();
//                break;
//        }
//    }
//
//    private void transitionTo(AutoState next) {
//        currentState = next;
//        pathTimer.resetTimer();
//    }
//
//    @Override
//    public void init() {
//        pathTimer = new Timer();
//        opmodeTimer = new Timer();
//
//        RobotHardware.init(hardwareMap);
//        follower = Constants.createFollower(hardwareMap);
//
//        intake = new IntakeAction(hardwareMap);
//        indexer = new IndexAction(hardwareMap);
//        ejector = new EjectorAction(hardwareMap);
//        outtake = new OuttakeAction(hardwareMap);
//
//        buildPaths();
//        follower.setStartingPose(startPose);
//    }
//
//    @Override
//    public void start() {
//        pathTimer.resetTimer();
//        currentState = AutoState.START;
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//        intake.runIn();
//        updateAutonomous();
//
//        telemetry.addLine("=== AutonR Debug ===");
//        telemetry.addData("State", currentState);
//        telemetry.addData("Follower Busy", follower.isBusy());
//        telemetry.addData("Timer (ms)", pathTimer.getElapsedTime());
//        telemetry.addLine("Pose:");
//        telemetry.addData("x", "%.2f", follower.getPose().getX());
//        telemetry.addData("y", "%.2f", follower.getPose().getY());
//        telemetry.addData("heading", "%.2f°", Math.toDegrees(follower.getPose().getHeading()));
//
//        // show shooter velocity if available
//        try {
//            double shooterVel = RobotHardware.outtakeMotor.getVelocity();
//            telemetry.addData("shooter TPS", "%.1f", shooterVel);
//            telemetry.addData("shooter goal", scoreShooterTPS);
//            telemetry.addData("atTarget", outtake.isAtTargetVelocity());
//            telemetry.addData("fire count", fireCount);
//            telemetry.addData("eject angle", ejector.paddle.getPosition());
//        } catch (Exception e) {
//            telemetry.addLine("Outtake motor velocity unavailable (check RobotHardware field).");
//        }
//
//        telemetry.update();
//    }
//}
