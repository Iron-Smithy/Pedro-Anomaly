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

public class AutonB_Close_P2G1_0 extends OpMode {
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

    private final long scoreShooterTPS = 1035;
    private final long tolerance = 25;

    private AutoFireTask fireTask = null; // A task is a program that uses multiple actions to perform an action sequence, this one contains the artifact shooting logic
    private ShooterAimTask aimTask; // this task contains the logic pertaining to outtake speed and turret direction

    private final double RPreXpos = 100;
    private final Pose startPoseRed = MConstants.goalResetPoseRed; // start location (up  against the goal)
    private final Pose ScorePoseRed = new Pose(100-4, 107+4, Math.toRadians(45)); // score location
    private final Pose R1PrePoseRed = new Pose(RPreXpos, 87, Math.toRadians(0)); // row 1 collection pre location
    private final Pose R1CollectPoseRed = new Pose(125, 87, Math.toRadians(0)); // row 1 balls inside robot location
    private final Pose ScoreR1CPPoseRed = new Pose(100, 87, Math.toRadians(0)); // smooth back-out bezier control point
    private final Pose R2PrePoseRed = new Pose(RPreXpos, 60, Math.toRadians(0)); // row 2 collection pre location
    private final Pose R2CollectPoseRed = new Pose(132, 60, Math.toRadians(0)); // row 2 balls inside robot location
    private final Pose ScoreR2CPPoseRed = new Pose(90, 55, Math.toRadians(0));  // smooth back-out bezier control point
    private final Pose ParkPoseRed = new Pose(125, 88.5, Math.toRadians(0)); // start pose, make right

    private final Pose gateHitPoseRed = new Pose(132, 70, Math.toRadians(0));
    private final Pose gateHitBackUpCPPoseRed = new Pose(122, 65, Math.toRadians(0));

    // gate auto: preload, 2nd spike, gate once, 1st spike
    // OR preload, 2nd spike but hit the lever , gate once, 1st spike
    // need a auto that hits the gate
    // gate: 130.5, 59.5, H35
    // then back up to 130.5, 55, H35

    private enum AutoState { // Auton step sequence
        START,
        GO_SCORE_PRELOAD,
        SCORE_PRELOAD,

        PICKUP_R1, // row 1
        GO_SCORE_R1,
        SCORE_R1,

        PICKUP_R2, // row 2
        HIT_GATE,
        HIT_GATE_HOLD,
        GO_SCORE_R2,
        SCORE_R2,


        EXIT,
        DONE
    }

//    private final Pose goalPose = pose(new Pose(132, 136, 0));
    private Pose goalPose;

    private AutoState currentState = AutoState.START;

    private PathChain startToScore;

    private PathChain row1Pickup;
    private PathChain row1Return;

    private PathChain row2Pickup;
    private PathChain row2toGateHit;
    private PathChain row2Return;

    private PathChain park;

    private void buildPaths() { // this function adapts poses to the proper alliance side, then generates the paths the robot should follow during autonomous
        Pose startPose = pose(startPoseRed);
        Pose scorePose = pose(ScorePoseRed);
        Pose r1Pre = pose(R1PrePoseRed);
        Pose r1Collect = pose(R1CollectPoseRed);
        Pose scoreR1CP = pose(ScoreR1CPPoseRed);
        Pose r2Pre = pose(R2PrePoseRed);
        Pose r2Collect = pose(R2CollectPoseRed);
        Pose scoreR2CP = pose(ScoreR2CPPoseRed);

        Pose gateHitPose = pose(gateHitPoseRed);
        Pose gateHitBackUpCPPose = pose(gateHitBackUpCPPoseRed);

        Pose parkPose = pose(ParkPoseRed);

        goalPose = pose(MConstants.goalPoseRed);

        // ========= START → SCORE =========
        startToScore = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose)) // addPath determines the xy travel
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading()) // setHeadingInterpolation will turn the robot during its path
                .build();

        // ========= ROW 1 =========
        row1Pickup = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, r1Pre))
                .setLinearHeadingInterpolation(scorePose.getHeading(), r1Pre.getHeading())
                .addPath(new BezierLine(r1Pre, r1Collect)) // a second path is added onto the last one making a path chain, multiple paths executed as one.
                .setConstantHeadingInterpolation(r1Collect.getHeading())
                .build();

        row1Return = follower.pathBuilder()
                .addPath(new BezierCurve(r1Collect, scoreR1CP, scorePose))
                .setLinearHeadingInterpolation(r1Collect.getHeading(), scorePose.getHeading())
                .build();

        // ========= ROW 2 =========
        row2Pickup = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, r2Pre))
                .setLinearHeadingInterpolation(scorePose.getHeading(), r2Pre.getHeading())
                .addPath(new BezierLine(r2Pre, r2Collect))
                .setConstantHeadingInterpolation(r2Collect.getHeading())
                .build();

        row2toGateHit = follower.pathBuilder()
                .addPath(new BezierCurve(r2Collect, gateHitBackUpCPPose, gateHitPose))
                .setConstantHeadingInterpolation(r2Collect.getHeading())
                .build();

        row2Return = follower.pathBuilder()
                .addPath(new BezierCurve(gateHitPose, scoreR2CP, scorePose))
                .setLinearHeadingInterpolation(gateHitPose.getHeading(), scorePose.getHeading())
                .build();

        // ========= Park =========
        park = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, r1Collect))
                .setLinearHeadingInterpolation(scorePose.getHeading(), r1Collect.getHeading())
                .build();
    }
    private void updateAutonomous() {
        Pose currentPose = follower.getPose();
        long outtakeSpeed = aimTask.getTargetSpeed(currentPose);

        switch (currentState) {
            // START
            case START:
                outtake.spinUp(scoreShooterTPS); // spin up outtake about the right speed for shooting
                RobotHardware.outtakeAngleAdjust.setPosition(MConstants.flapUp);

                follower.followPath(startToScore); // follow the path startToScore
                transitionTo(AutoState.GO_SCORE_PRELOAD); // then go to the next logic step
                break;

            // SCORE PRELOAD
            case GO_SCORE_PRELOAD:
                outtake.spinUp(outtakeSpeed); // set speed based on current distance from goal
                aimAtTarget(currentPose); // aim turret at the goal

                if (!follower.isBusy()) { // once the robot has reached the target position
                    blocker.in();
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
                    follower.followPath(row2Pickup, true);
                    transitionTo(AutoState.PICKUP_R2);
                    fireTask = null; // destroy old firetask
                    blocker.out(); // prevent balls from exiting early
                }
                break;
            // SCORE R2
            case PICKUP_R2:
                if (!follower.isBusy()) {
                    intake.runInAt(0.50);
//                    intake.stop();
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
                    follower.followPath(row2Return, 0.7, false);
                    transitionTo(AutoState.GO_SCORE_R2);
                }
                break;
            case GO_SCORE_R2:
                outtake.spinUp(outtakeSpeed);
                aimAtTarget(currentPose);

                if (!follower.isBusy()) {
                    blocker.in();
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
                    follower.followPath(row1Pickup, true); // row3Pickup
                    transitionTo(AutoState.PICKUP_R1); // PICKUP_R3
                    blocker.out();
                    fireTask = null;
                }
                break;
            // SCORE R1
            case PICKUP_R1:
                if (!follower.isBusy()) { // once finished collecting objects
                    indexer.runInAt(0.3); // indexer spin in (speed changer don't work :( )
                    intake.runInAt(0.45); // slow intake to put less pressure on intake, motors, blockers etc. but keep spinning to catch balls not fully in
                    follower.followPath(row1Return, true);
                    transitionTo(AutoState.GO_SCORE_R1);
                }
                break;
            case GO_SCORE_R1:
                outtake.spinUp(outtakeSpeed);
                aimAtTarget(currentPose);

                if (!follower.isBusy()) {
                    blocker.in();
                    fireTask = new AutoFireTask(outtake, indexer, ejector, intake, ballSensors, scoreShooterTPS);
                    fireTask.start();
                    transitionTo(AutoState.SCORE_R1);
                }
                break;
            case SCORE_R1:
                outtake.spinUp(outtakeSpeed);
                aimAtTarget(currentPose);
                fireTask.update(outtakeSpeed);

                if (fireTask.isActive()) {
                } else {
                    intake.runIn();
                    follower.followPath(park, true);
                    transitionTo(AutoState.EXIT);
                    fireTask = null;
                    blocker.out();
                }
                break;

            // PARK
            case EXIT:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 1000) { // once reached path and enough time has passed for robot to have settled
                    transitionTo(AutoState.DONE);
                }
                break;

            case DONE: // WE'RE DONE! : D
                outtake.stop(); // turn off all components
                indexer.stop();
                intake.stop();
                ejector.down();
                blocker.out();
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