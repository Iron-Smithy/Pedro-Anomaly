//package org.firstinspires.ftc.teamcode.pedroPathing.Paths.BLUE;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.teamcode.RobotHardware;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//import org.firstinspires.ftc.teamcode.pedroPathing.MConstants;
//import org.firstinspires.ftc.teamcode.tools.Actions.AutoFireTask;
//import org.firstinspires.ftc.teamcode.tools.Actions.BlockerAction;
//import org.firstinspires.ftc.teamcode.tools.Actions.EjectorAction;
//import org.firstinspires.ftc.teamcode.tools.Actions.IndexAction;
//import org.firstinspires.ftc.teamcode.tools.Actions.IntakeAction;
//import org.firstinspires.ftc.teamcode.tools.Actions.OuttakeAction;
//import org.firstinspires.ftc.teamcode.tools.Actions.TurretAction;
//import org.firstinspires.ftc.teamcode.tools.Sensors.BallSensorArray;
//
//@Autonomous(name = "Auton_BLUE_12P", group = "AAA")
//public class Auton_BLUE_12P_C extends OpMode {
//    private Follower follower;
//    private Timer pathTimer, opmodeTimer;
//
//    private IntakeAction intake;
//    private IndexAction indexer;
//    private EjectorAction ejector;
//    private OuttakeAction outtake;
//    private TurretAction turret;
//    private BlockerAction blocker;
//
//    private BallSensorArray ballSensors;
//
//    private final long scoreShooterTPS = 1035;
//
//    private final long tolerance = 25;
//
//    private AutoFireTask fireTask = null;
//
//    private final double RPreXpos = 85;
//    private final Pose startPose = new Pose(144-120, 127, Math.toRadians(180-37)); // start location
//    private final Pose ScorePose = new Pose(144-100, 107, Math.toRadians(180-45)); // score location
//    private final Pose R1PrePose = new Pose(144-RPreXpos, 87, Math.toRadians(180-0)); // row 1 collection pre location
//    private final Pose R1CollectPose = new Pose(144-123, 87, Math.toRadians(180-0)); // row 1 balls inside robot location
//    private final Pose ScoreR1CPPose = new Pose(144-100, 87, Math.toRadians(180-0)); // smooth back-out bezier control point
//    private final Pose R2PrePose = new Pose(144-RPreXpos, 63, Math.toRadians(180-0)); // row 2 collection pre location
//    private final Pose R2CollectPose = new Pose(144-125, 63, Math.toRadians(180-0)); // row 3 balls inside robot location
//    private final Pose ScoreR2CPPose = new Pose(144-90, 55, Math.toRadians(180-0));  // smooth back-out bezier control point
//    private final Pose R3PrePose = new Pose(144-RPreXpos, 39, Math.toRadians(180-0)); // row 3 collection pre location
//    private final Pose R3CollectPose = new Pose(144-135, 39, Math.toRadians(180-0)); // row 3 balls inside robot location
//    private final Pose ScoreR3CPPose = new Pose(144-90, 30, Math.toRadians(180-0));  // smooth back-out bezier control point
//    private final Pose ParkPose = new Pose(144-120, 127, Math.toRadians(180-37)); // start pose, make right
//
//    private enum AutoState {
//        START,
//        GO_SCORE_PRELOAD,
//        SCORE_PRELOAD,
//
//        PICKUP_R1, // row 1
//        GO_SCORE_R1,
//        SCORE_R1,
//
//        PICKUP_R2, // row 2
//        GO_SCORE_R2,
//        SCORE_R2,
//
//        PICKUP_R3, // row 3
//        GO_SCORE_R3,
//        SCORE_R3,
//
//        EXIT,
//        DONE
//    }
//
//    private AutoState currentState = AutoState.START;
//
//    private PathChain startToScore;
//
//    private PathChain row1Pickup;
//    private PathChain row1Return;
//
//    private PathChain row2Pickup;
//    private PathChain row2Return;
//
//    private PathChain row3Pickup;
//    private PathChain row3Return;
//
//    private PathChain park;
//
//    private void buildPaths() {
//        // ========= START → SCORE =========
//        startToScore = follower.pathBuilder()
//                .addPath(new BezierLine(startPose, ScorePose))
//                .setLinearHeadingInterpolation(startPose.getHeading(), ScorePose.getHeading())
//                .setVelocityConstraint(0.5)
//                .build();
//
//        // ========= ROW 1 =========
//        row1Pickup = follower.pathBuilder()
//                .addPath(new BezierLine(ScorePose, R1PrePose))
//                .setLinearHeadingInterpolation(ScorePose.getHeading(), R1PrePose.getHeading())
//                .addPath(new BezierLine(R1PrePose, R1CollectPose))
//                .setConstantHeadingInterpolation(R1CollectPose.getHeading())
//                .setVelocityConstraint(0.1)
//                .build();
//
//        row1Return = follower.pathBuilder()
//                // You can still use control points by using the addPath method that takes multiple Poses
//                .addPath(new BezierCurve(R1CollectPose, ScoreR1CPPose, ScorePose))
//                .setLinearHeadingInterpolation(R1CollectPose.getHeading(), ScorePose.getHeading())
//                .setVelocityConstraint(0.6)
//                .build();
//
//        // ========= ROW 2 =========
//        row2Pickup = follower.pathBuilder()
//                .addPath(new BezierLine(ScorePose, R2PrePose))
//                .setLinearHeadingInterpolation(ScorePose.getHeading(), R2PrePose.getHeading())
//                .addPath(new BezierLine(R2PrePose, R2CollectPose))
//                .setConstantHeadingInterpolation(R2CollectPose.getHeading())
//                .setVelocityConstraint(0.1)
//                .build();
//
//        row2Return = follower.pathBuilder()
//                .addPath(new BezierCurve(R2CollectPose, ScoreR2CPPose, ScorePose))
//                .setLinearHeadingInterpolation(R2CollectPose.getHeading(), ScorePose.getHeading())
//                .setVelocityConstraint(0.6)
//                .build();
//
//        // ========= ROW 3 =========
//        row3Pickup = follower.pathBuilder()
//                .addPath(new BezierLine(ScorePose, R3PrePose))
//                .setLinearHeadingInterpolation(ScorePose.getHeading(), R3PrePose.getHeading())
//                .addPath(new BezierLine(R3PrePose, R3CollectPose))
//                .setConstantHeadingInterpolation(R3CollectPose.getHeading())
//                .setVelocityConstraint(0.1)
//                .build();
//
//        row3Return = follower.pathBuilder()
//                .addPath(new BezierCurve(R3CollectPose, ScoreR3CPPose, ScorePose))
//                .setLinearHeadingInterpolation(R3CollectPose.getHeading(), ScorePose.getHeading())
//                .setVelocityConstraint(0.6)
//                .build();
//
//        // ========= Park =========
//        park = follower.pathBuilder()
//                .addPath(new BezierLine(ScorePose, R1CollectPose))
//                .setLinearHeadingInterpolation(ScorePose.getHeading(), R1CollectPose.getHeading())
//                .setVelocityConstraint(0.7)
//                .build();
////              .addPath(new BezierLine(ScorePose, startPose))
////              .setLinearHeadingInterpolation(ScorePose.getHeading(), ParkPose.getHeading())
////              .setVelocityConstraint(0.6)
////              .build();
//    }
//
//
//
//    private void updateAutonomous() {
//        switch (currentState) {
//            // START
//            case START:
//                outtake.spinUp(scoreShooterTPS);
//                RobotHardware.outtakeAngleAdjust.setPosition(MConstants.flapUp);
//
//                follower.setMaxPower(0.65);
//
//                follower.followPath(startToScore);
//                transitionTo(AutoState.GO_SCORE_PRELOAD);
//                break;
//
//            // SCORE PRELOAD
//            case GO_SCORE_PRELOAD:
//                if (!follower.isBusy()) {
//                    blocker.in();
////                    fireTask = new AutoFireTask(outtake, indexer, ejector, intake, scoreShooterTPS); // to take advantage of our brand new
//                    fireTask = new AutoFireTask(outtake, indexer, ejector, intake, ballSensors, scoreShooterTPS);
//                    fireTask.start();
//                    transitionTo(AutoState.SCORE_PRELOAD);
//                }
//                break;
//
//            case SCORE_PRELOAD:
//                fireTask.update();
//                if (fireTask.isActive()) {
//                } else {
//                    intake.runIn();
//                    follower.followPath(row1Pickup, 0.8, true);
//                    transitionTo(AutoState.PICKUP_R1);
//                    fireTask = null;
//                    blocker.out();
//                }
//                break;
//
//            // SCORE R1
//            case PICKUP_R1:
//                if (!follower.isBusy()) {
//                    indexer.runInAt(0.3);
//                    intake.runInAt(0.45);
//                    follower.followPath(row1Return, 0.7, true);
//                    transitionTo(AutoState.GO_SCORE_R1);
//                }
//                break;
//            case GO_SCORE_R1:
//                if (!follower.isBusy()) {
//                    blocker.in();
////                    fireTask = new AutoFireTask(outtake, indexer, ejector, intake, scoreShooterTPS);
//                    fireTask = new AutoFireTask(outtake, indexer, ejector, intake, ballSensors, scoreShooterTPS);
//                    fireTask.start();
//                    transitionTo(AutoState.SCORE_R1);
//                }
//                break;
//            case SCORE_R1:
//                fireTask.update();
//                if (fireTask.isActive()) {
//                } else {
//                    intake.runIn();
//                    follower.followPath(row2Pickup, 0.8, true);
//                    transitionTo(AutoState.PICKUP_R2);
//                    fireTask = null;
//                    blocker.out();
//                }
//                break;
//
//            // SCORE R2
//            case PICKUP_R2:
//                if (!follower.isBusy()) {
//                    intake.runInAt(0.25);
////                    intake.stop();
//                    follower.followPath(row2Return, 0.7, true);
//                    transitionTo(AutoState.GO_SCORE_R2);
//                }
//                break;
//            case GO_SCORE_R2:
//                if (!follower.isBusy()) {
//                    blocker.in();
////                    fireTask = new AutoFireTask(outtake, indexer, ejector, intake, scoreShooterTPS);
//                    fireTask = new AutoFireTask(outtake, indexer, ejector, intake, ballSensors, scoreShooterTPS);
//                    fireTask.start();
//                    transitionTo(AutoState.SCORE_R2);
//                }
//                break;
//            case SCORE_R2:
//                fireTask.update();
//                if (fireTask.isActive()) {
//                } else {
//                    intake.runIn();
//                    follower.followPath(row3Pickup, 0.8, true); // row3Pickup
//                    transitionTo(AutoState.PICKUP_R3); // PICKUP_R3
//                    blocker.out();
//                    fireTask = null;
//                }
//                break;
//
//            // SCORE R3
//            case PICKUP_R3:
//                if (!follower.isBusy()) {
//                    intake.stop();
//                    follower.followPath(row3Return, 0.7, true); // row3Return
//                    transitionTo(AutoState.GO_SCORE_R3); //GO_SCORE_R3
//                }
//                break;
//            case GO_SCORE_R3:
//                if (!follower.isBusy()) {
//                    blocker.in();
////                    fireTask = new AutoFireTask(outtake, indexer, ejector, intake, scoreShooterTPS);
//                    fireTask = new AutoFireTask(outtake, indexer, ejector, intake, ballSensors, scoreShooterTPS);
//                    fireTask.start();
//                    transitionTo(AutoState.SCORE_R3);
//                }
//                break;
//            case SCORE_R3:
//                fireTask.update();
//                if (fireTask.isActive()) {
//                } else {
//                    fireTask = null;
//                    follower.followPath(park, 0.9, true);
//                    transitionTo(AutoState.EXIT);
//                }
//                break;
//
//            // PARK
//            case EXIT:
//                if (!follower.isBusy() && pathTimer.getElapsedTime() > 1000) {
//                    outtake.stop();
//                    indexer.stop();
//                    intake.stop();
//                    ejector.down();
//                    blocker.out();
//                    transitionTo(AutoState.DONE);
//                }
//                break;
//
//            case DONE:
//                break;
//
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
//        turret = new TurretAction(hardwareMap);
//        blocker = new BlockerAction(hardwareMap);
//
//        ballSensors = new BallSensorArray();
//
//        turret.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // reset to 0
//        turret.runToTick((int) turret.center); // hold at 0
//
//        ejector.up();
//        ejector.down();
//
//        blocker.out();
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
//
//        outtake.spinUp(scoreShooterTPS);
//        outtake.update();
//
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
////            telemetry.addData("fire count", fireCount);
//            telemetry.addData("eject angle", ejector.paddle.getPosition());
//        } catch (Exception e) {
//            telemetry.addLine("Outtake motor velocity unavailable (check RobotHardware field).");
//        }
//
//        telemetry.update();
//    }
//}