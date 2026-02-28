package org.firstinspires.ftc.teamcode.pedroPathing.Auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Actions.BlockerAction;
import org.firstinspires.ftc.teamcode.Actions.EjectorAction;
import org.firstinspires.ftc.teamcode.Actions.IndexAction;
import org.firstinspires.ftc.teamcode.Actions.IntakeAction;
import org.firstinspires.ftc.teamcode.Actions.OuttakeAction;
import org.firstinspires.ftc.teamcode.Actions.TurretAction;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.Sensors.BallSensorArray;
import org.firstinspires.ftc.teamcode.Tasks.AutoFireTask;
import org.firstinspires.ftc.teamcode.Tasks.ShooterAimTask;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.MConstants;

public class AutonB_Far_P32GH_0 extends OpMode {
    private Alliance alliance = Alliance.RED; // defualt

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }


    private Follower follower; // PedroPathing path calculator/solver
    private Timer pathTimer, opmodeTimer; // it's a timer

    private IntakeAction intake; // each action is a program that handles its respective robot mechanism
    private IndexAction indexer;
    private EjectorAction ejector;
    private OuttakeAction outtake;
    private TurretAction turret;
    private BlockerAction blocker;
    private BallSensorArray ballSensors;

    private final long scoreShooterTPS = 1444;
    private final long tolerance = 25;

    private AutoFireTask fireTask = null; // A task is a program that uses multiple actions to perform an action sequence, this one contains the artifact shooting logic
    private ShooterAimTask aimTask; // this task contains the logic pertaining to outtake speed and turret direction

    private final Pose startPoseRed = new Pose(96, 7, Math.toRadians(90)); // start location
    private final Pose ScorePoseRed = new Pose(90, 13, Math.toRadians(45)); // score location
    private final Pose R3PrePoseRed = new Pose(99, 36, Math.toRadians(0)); // row 3 collection pre location
    private final Pose R3CollectPoseRed = new Pose(125, 36, Math.toRadians(0)); // row 3 balls inside robot location
    private final Pose R2PrePoseRed = new Pose(99, 60, Math.toRadians(0)); // row 2 collection pre location
    private final Pose R2CollectPoseRed = new Pose(125, 60, Math.toRadians(0)); // row 2 balls inside robot location
    private final Pose gateHitPoseRed = new Pose(124, 70, Math.toRadians(0));
    private final Pose gateHitBackUpCPPoseRed = new Pose(115, 65, Math.toRadians(0));
    private final Pose RHumanRed = new Pose(125, 9, Math.toRadians(0));
    private final Pose backUpRed = new Pose(133, 9, Math.toRadians(0)); //so can re-smash into wall
    private final Pose FarParkPoseRed = new Pose(110, 15, Math.toRadians(0));
    private Pose goalPose;



    private enum AutoState {
        START,
        GO_SCORE_PRELOAD,
        SCORE_PRELOAD,

        PICKUP_R3, // row 3
        GO_SCORE_R3,
        SCORE_R3,

        PICKUP_R2, // row 2
        HIT_GATE,
        HIT_GATE_HOLD,
        GO_SCORE_R2,
        SCORE_R2,

        PICKUP_RHuman, // row H
        GO_SCORE_RHuman,
        SCORE_RHuman,

        EXIT,
        DONE
    }

    private AutoState currentState = AutoState.START;

    private PathChain startToScore;

    private PathChain row3Pickup;
    private PathChain row3Return;

    private PathChain row2Pickup;
    private PathChain row2toGateHit;
    private PathChain row2Return;

    private PathChain rowRHPickup;
    private PathChain rowRHReturn;

    private PathChain park;

    private void buildPaths() { // this function adapts poses to the proper alliance side, then generates the paths the robot should follow during autonomous
        Pose startPose = pose(startPoseRed);
        Pose scorePose = pose(ScorePoseRed);
        Pose r3Pre = pose(R3PrePoseRed);
        Pose r3Collect = pose(R3CollectPoseRed);
        Pose r2Pre = pose(R2PrePoseRed);
        Pose r2Collect = pose(R2CollectPoseRed);
        Pose RHuman = pose(RHumanRed);
        Pose backUp = pose(backUpRed);
        Pose FarPark = pose(FarParkPoseRed);

        Pose gateHitPose = pose(gateHitPoseRed);
        Pose gateHitBackUpCPPose = pose(gateHitBackUpCPPoseRed);

        goalPose = pose(new Pose(140, 135, 0));


        // ========= START → SCORE =========
        startToScore = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose)) // addPath determines the xy travel
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading()) // setHeadingInterpolation will turn the robot during its path
                .build();

        // ========= ROW 3 =========
        row3Pickup = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, r3Pre))
                .setLinearHeadingInterpolation(scorePose.getHeading(), r3Pre.getHeading())
                .addPath(new BezierLine(r3Pre, r3Collect)) // a second path is added onto the last one making a path chain, multiple paths executed as one.
                .setConstantHeadingInterpolation(r3Collect.getHeading())
                .build();

        row3Return = follower.pathBuilder()
                .addPath(new BezierLine(r3Collect, scorePose))
                .setLinearHeadingInterpolation(r3Collect.getHeading(), scorePose.getHeading())
                .build();

        // ========= ROW 2 =========
        row2Pickup = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, r2Pre))
                .setLinearHeadingInterpolation(scorePose.getHeading(), r2Pre.getHeading())
                .addPath(new BezierLine(r2Pre, r2Collect)) // a second path is added onto the last one making a path chain, multiple paths executed as one.
                .setConstantHeadingInterpolation(r2Collect.getHeading())
                .build();

        row2toGateHit = follower.pathBuilder()
                .addPath(new BezierCurve(r2Collect, gateHitBackUpCPPose, gateHitPose))
                .setConstantHeadingInterpolation(r2Collect.getHeading())
                .build();

        row2Return = follower.pathBuilder()
                .addPath(new BezierLine(gateHitPose, scorePose))
                .setLinearHeadingInterpolation(gateHitPose.getHeading(), scorePose.getHeading())
                .build();

        // ========= ROW RH =========
        rowRHPickup = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, RHuman))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(RHuman, backUp))
                .setConstantHeadingInterpolation(RHuman.getHeading())
                .addPath(new BezierLine(backUp, RHuman))
                .setConstantHeadingInterpolation(RHuman.getHeading())
                .build();

        rowRHReturn = follower.pathBuilder()
                .addPath(new BezierLine(RHuman, scorePose))
                .setLinearHeadingInterpolation(RHuman.getHeading(), scorePose.getHeading())
                .build();

        // ========= Park =========
        park = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, FarPark))
                .setLinearHeadingInterpolation(scorePose.getHeading(), FarPark.getHeading())
                .build();
    }
    private void updateAutonomous() {
        Pose currentPose = follower.getPose();
        long outtakeSpeed = aimTask.getTargetSpeed(currentPose);

        switch (currentState) {
            // START
            case START:
                outtake.spinUp(scoreShooterTPS); // spin up outtake about the right speed for shooting
                RobotHardware.outtakeAngleAdjust.setPosition(MConstants.flapDown);

                follower.followPath(startToScore, 0.7, true); // follow the path startToScore
                transitionTo(AutoState.GO_SCORE_PRELOAD); // then go to the next logic step
                break;

            // SCORE PRELOAD
            case GO_SCORE_PRELOAD:
                outtake.spinUp(outtakeSpeed); // set speed based on current distance from goal
                aimAtTarget(currentPose); // aim turret at the goal

                if (!follower.isBusy()) { // once the robot has reached the target position
                    blocker.in();
                    indexer.runInAt(0.3); // Added this in case not indexing fast?
                    fireTask = new AutoFireTask(outtake, indexer, ejector, intake, ballSensors, scoreShooterTPS); // set up a new shooting program
                    fireTask.start(); // start shooting
                    transitionTo(AutoState.SCORE_PRELOAD); // go to next logic step
                }
                break;

            case SCORE_PRELOAD:
                outtake.spinUp(outtakeSpeed); // keep updating speed based on distance in case the robot still moving
                aimAtTarget(currentPose); // keep updating aim in case the robot still moving
                fireTask.update(outtakeSpeed); // update the shooting program so it can fire

                if (fireTask.isActive()) {
                } else { // once firetask has finished
                    intake.runIn(); // turn on intake to get ready to intake
                    follower.followPath(row3Pickup, true);
                    transitionTo(AutoState.PICKUP_R3);
                    fireTask = null; // destroy old firetask
                    blocker.out(); // prevent balls from exiting early
                }
                break;

            // SCORE R3
            case PICKUP_R3:
                if (!follower.isBusy()) { // once finished collecting objects
//                    indexer.runInAt(0.3); // indexer spin in (speed changer don't work :( )
                    indexer.stop(); // to not drain battery
                    intake.runInAt(0.45);
                    follower.followPath(row3Return, 0.7, true);
                    transitionTo(AutoState.GO_SCORE_R3);
                }
                break;
            case GO_SCORE_R3:
                outtake.spinUp(outtakeSpeed);
                aimAtTarget(currentPose);

                if (!follower.isBusy()) {
                    blocker.in();
                    indexer.runInAt(0.3); // just in case
                    fireTask = new AutoFireTask(outtake, indexer, ejector, intake, ballSensors, scoreShooterTPS);
                    fireTask.start();
                    transitionTo(AutoState.SCORE_R3);
                }
                break;
            case SCORE_R3:
                outtake.spinUp(outtakeSpeed);
                aimAtTarget(currentPose);
                fireTask.update(outtakeSpeed);

                if (fireTask.isActive()) {
                } else {
                    intake.runIn();
                    follower.followPath(row2Pickup, true);
                    transitionTo(AutoState.PICKUP_R2);
                    fireTask = null;
                    blocker.out();
                }
                break;

            // SCORE R2
            case PICKUP_R2:
                if (!follower.isBusy()) { // once finished collecting objects
//                    indexer.runInAt(0.3); // indexer spin in (speed changer don't work :( )
                    indexer.stop(); // to not drain battery
                    intake.runInAt(0.3); // was .45. slow intake to put less pressure on intake, motors, blockers etc. but keep spinning to catch balls not fully in
                    follower.followPath(row2toGateHit, true);
                    transitionTo(AutoState.HIT_GATE);
                }
                break;
            case HIT_GATE:
                if (!follower.isBusy()) { // once finished hitting gate
                    indexer.stop(); // to not drain battery
//                    intake.runInAt(0.3); // was .45. slow intake to put less pressure on intake, motors, blockers etc. but keep spinning to catch balls not fully in
                    intake.stop(); // added this
                    transitionTo(AutoState.HIT_GATE_HOLD);
                } else if (pathTimer.getElapsedTime() > 3000) {
                    indexer.stop(); // to not drain battery
//                    intake.runInAt(0.3);
                    follower.followPath(row2Return, 0.7, true);
                    transitionTo(AutoState.GO_SCORE_R2);
                }
                break;
            case HIT_GATE_HOLD:
                if (pathTimer.getElapsedTime() > 1000) {
                    follower.followPath(row2Return, 0.7, true);
                    transitionTo(AutoState.GO_SCORE_R2);
                }
                break;
            case GO_SCORE_R2:
                outtake.spinUp(outtakeSpeed);
                aimAtTarget(currentPose);

                if (!follower.isBusy()) {
                    blocker.in();
                    indexer.runInAt(0.3); // just in case
                    fireTask = new AutoFireTask(outtake, indexer, ejector, intake, ballSensors, scoreShooterTPS);
                    fireTask.start();
                    transitionTo(AutoState.SCORE_R2);
                }
                break;
            case SCORE_R2:
                outtake.spinUp(outtakeSpeed);
                aimAtTarget(currentPose);
                fireTask.update(outtakeSpeed);

                if (fireTask.isActive()) {
                } else {
                    intake.runIn();
                    follower.followPath(rowRHPickup, true);
                    transitionTo(AutoState.PICKUP_RHuman);
                    fireTask = null;
                    blocker.out();
                }
                break;

            // SCORE RHuman
            case PICKUP_RHuman:
                if (!follower.isBusy() || pathTimer.getElapsedTime() > 3000) {
//                    intake.runInAt(0.50);
                    intake.stop(); // to leave behind any extra balls
                    indexer.stop(); // to not drain battery
                    follower.followPath(rowRHReturn, 0.7, true);
                    transitionTo(AutoState.GO_SCORE_RHuman);
                }
                break;
            case GO_SCORE_RHuman:
                outtake.spinUp(outtakeSpeed);
                aimAtTarget(currentPose);

                if (!follower.isBusy()) {
                    blocker.in();
                    indexer.runInAt(0.3); // just in case
                    fireTask = new AutoFireTask(outtake, indexer, ejector, intake, ballSensors, scoreShooterTPS);
                    fireTask.start();
                    transitionTo(AutoState.SCORE_RHuman);
                }
                break;
            case SCORE_RHuman:
                outtake.spinUp(outtakeSpeed);
                aimAtTarget(currentPose);
                fireTask.update(outtakeSpeed);

                if (fireTask.isActive()) {
                } else {
                    intake.runIn();
                    follower.followPath(park, true); // scoreToPark
                    transitionTo(AutoState.EXIT); // EXIT
                    blocker.out();
                    fireTask = null;
                }
                break;

            // PARK
            case EXIT:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 1000) { // once reached path and enough time has passed for robot to have settled
                    outtake.stop(); // turn off all components
                    indexer.stop();
                    intake.stop();
                    ejector.down();
                    blocker.out();
                    transitionTo(AutoState.DONE);
                }
                break;

            case DONE: // WE'RE DONE! : D
                break;
        }
    }

    private void transitionTo(AutoState next) {
        currentState = next;
        pathTimer.resetTimer();
    }

    private Pose pose(Pose redPose) {
        if (alliance == Alliance.RED) {
            return redPose;
        } else {
            return redPose.mirror();
        }
    }

    @Override
    public void init() { // set up code
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        RobotHardware.init(hardwareMap); // get all hardware components
        follower = Constants.createFollower(hardwareMap); // give follower control of the wheels

        intake = new IntakeAction(hardwareMap); // give actions control of their respective components
        indexer = new IndexAction(hardwareMap);
        ejector = new EjectorAction(hardwareMap);
        outtake = new OuttakeAction(hardwareMap);
        turret = new TurretAction(hardwareMap);
        blocker = new BlockerAction(hardwareMap);

        ballSensors = new BallSensorArray();

        turret.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // reset to 0

        ejector.up(); // I don't know why this needs to be done but if not, ejector don't work the first time we call it ¯\_(ツ)_/¯
        ejector.down();

        blocker.out();

        buildPaths();
        Pose startPose = pose(startPoseRed);
        follower.setStartingPose(startPose);

        // ALL OBJECTS that need a pose needs to be called AFTER poses are built
        aimTask = new ShooterAimTask(goalPose, ShooterAimTask.speedMap()); // give aim task the goal location and the speed map
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        currentState = AutoState.START;

        turret.runToTick((int) turret.center); // keep at zero until ready
    }

    @Override
    public void loop() {
        follower.update(); // move robot based on set pathing

        updateAutonomous(); // check logic tree
        outtake.update(); // adjust speed

        telemetry.addData("Alliance", alliance);
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
            telemetry.addData("shooter goal", outtake.targetVelocity);
            telemetry.addData("is at target?", outtake.isAtTargetVelocity());
//            telemetry.addData("fire count", fireCount);
            telemetry.addData("eject angle", ejector.paddle.getPosition());
        } catch (Exception e) {
            telemetry.addLine("Outtake motor velocity unavailable (check RobotHardware field).");
        }

        telemetry.update();
    }
    public void aimAtTarget(Pose currentPose) {
        Pose goalPose = aimTask.getGoalPose();
        double xDiff = (goalPose.getX() - currentPose.getX());
        double yDiff = (goalPose.getY() -  currentPose.getY());
        double targetAngle = Math.atan2(yDiff, xDiff);
        double turretError = targetAngle - follower.getHeading();
        double wrappedAngle = turret.calculateWrapAngles(turretError);
        double limitedAngle = turret.limitRadValues(wrappedAngle);
        int tickRaw = turret.radToTick(limitedAngle);
        turret.runToTick(tickRaw);
    }
}