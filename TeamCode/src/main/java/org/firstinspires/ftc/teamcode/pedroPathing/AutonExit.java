package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.tools.Actions.EjectorAction;
import org.firstinspires.ftc.teamcode.tools.Actions.IndexAction;
import org.firstinspires.ftc.teamcode.tools.Actions.IntakeAction;
import org.firstinspires.ftc.teamcode.tools.Actions.OuttakeAction;

@Autonomous(name = "AutonExit", group = "AAA")
public class AutonExit extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    private IntakeAction intake;
    private IndexAction indexer;
    private EjectorAction ejector;
    private OuttakeAction outtake;

    private long scoreShooterTPS = 1150;
    private long tolerance = 50;

    private long fireCount = 0;

    private final Pose startPose = new Pose(95, 6, Math.toRadians(90));
    private final Pose parkPose = new Pose(95, 40, Math.toRadians(90));

    private Path park;

    private enum AutoState {
        START,
        NEXT,
        DONE
    }

    private AutoState currentState = AutoState.START;

    private void buildPaths() {
        park = new Path(new BezierLine(startPose, parkPose));
        park.setLinearHeadingInterpolation(startPose.getHeading(), parkPose.getHeading());
        park.setVelocityConstraint(0.8);
    }

    private void updateAutonomous() {
        switch (currentState) {
            case START:
                follower.followPath(park);
                transitionTo(AutoState.NEXT);
                break;
            case NEXT:
                if ((!follower.isBusy() || pathTimer.getElapsedTime() >= 8000)) {
                    transitionTo(AutoState.DONE);
                }
                break;

            case DONE:
                outtake.stop();
                indexer.stop();
                intake.stop();
                ejector.down();
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
