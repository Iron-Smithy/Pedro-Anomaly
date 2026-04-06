package org.firstinspires.ftc.teamcode.pedroPathing.Auton;

import com.pedropathing.follower.Follower;
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


public class AutonB_Far_LEAVE_0 extends OpMode {
    private Alliance alliance = Alliance.RED; // defualt

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }


    private Follower follower; // PedroPathing path calculator/solver
    private Timer pathTimer, opmodeTimer; // it's a timer

    private int rowHumanCompletionCount = 0;

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

    private final Pose startPoseRed = new Pose(98.5, 10.5, Math.toRadians(90)); // start location   98.5, 10.5,   +2.5, +3.5
    private final Pose ScorePoseRed = new Pose(92.5, 16.5, Math.toRadians(45)); // score location
    private final Pose R3PrePoseRed = new Pose(101.5, 39.5, Math.toRadians(0)); // row 3 collection pre location
    private final Pose R3CollectPoseRed = new Pose(130.5, 39.5, Math.toRadians(0)); // row 3 balls inside robot location
    private final Pose R2PrePoseRed = new Pose(101.5, 63.5, Math.toRadians(0)); // row 2 collection pre location
    private final Pose R2CollectPoseRed = new Pose(130.5, 63.5, Math.toRadians(0)); // row 2 balls inside robot location
    private final Pose gateHitPoseRed = new Pose(126.5, 73.5, Math.toRadians(0)); //was 125.5, 73.5,
    private final Pose gateHitBackUpCPPoseRed = new Pose(117.5, 68.5, Math.toRadians(0));
    private final Pose RHumanRed = new Pose(127.5, 12.5, Math.toRadians(0));
    private final Pose backUpRed = new Pose(122.5, 12.5, Math.toRadians(0)); //so can re-smash into wall
    private final Pose RHumanRed2 = new Pose(129.5, 12.5, Math.toRadians(0));
    private final Pose FarParkPoseRed = new Pose(112.5, 13.5, Math.toRadians(0));
    private Pose goalPose;



    private enum AutoState {
        START,

        EXIT,
        DONE
    }

    private AutoState currentState = AutoState.START;
    private PathChain startToPark;

    private void buildPaths() { // this function adapts poses to the proper alliance side, then generates the paths the robot should follow during autonomous
        Pose startPose = pose(startPoseRed);
        Pose FarPark = pose(FarParkPoseRed);

//        goalPose = pose(MConstants.goalPoseRed); //142, 142
        goalPose = pose(new Pose(140, 142, 0));


        // ========= START → Park =========
        startToPark = follower.pathBuilder()
                .addPath(new BezierLine(startPose, FarPark)) // addPath determines the xy travel
                .setLinearHeadingInterpolation(startPose.getHeading(), FarPark.getHeading()) // setHeadingInterpolation will turn the robot during its path
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

                follower.followPath(startToPark, 1, true); // was 0.7    // follow the path startToScore
                transitionTo(AutoState.EXIT); // then go to the next logic step
                break;

            // PARK
            case EXIT:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 1000) { //RobotUtils.isStable(follower)     // once reached path and enough time has passed for robot to have settled
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
        turret.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        aimAtTarget(follower.getPose()); // aim turret at the goal
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