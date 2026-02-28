package org.firstinspires.ftc.teamcode.Tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

// CONSTANTS
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Auton.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.MConstants;

// ACTIONS
import org.firstinspires.ftc.teamcode.Actions.BlockerAction;
import org.firstinspires.ftc.teamcode.Actions.EjectorAction;
import org.firstinspires.ftc.teamcode.Actions.IndexAction;
import org.firstinspires.ftc.teamcode.Actions.IntakeAction;
import org.firstinspires.ftc.teamcode.Actions.OuttakeAction;
import org.firstinspires.ftc.teamcode.Actions.TurretAction;

// SENSORS
import org.firstinspires.ftc.teamcode.Sensors.BallPosition;
import org.firstinspires.ftc.teamcode.Sensors.BallSensorArray;

// TASKS
import org.firstinspires.ftc.teamcode.Tasks.DriveTask;
import org.firstinspires.ftc.teamcode.Tasks.ShooterAimTask;
import org.firstinspires.ftc.teamcode.Tasks.AutoFireTask;

@Configurable
@TeleOp (name = "Base Controller", group = "LinearOpMode")
public class BaseController extends OpMode {
    private Alliance alliance = Alliance.RED; // defualt

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    private DriveTask driveTask;
    private Follower follower;

    // ----- Telemetry -----
    private TelemetryManager telemetryM;

    private Pose startPose;
    private Pose goalRESET;
    private Pose humanRESET;
    private Pose goalPose;

    private void doPoseMath() {
        startPose = pose(new Pose(72, 137, Math.toRadians(270)));
        goalRESET = pose(MConstants.goalResetPoseRed);
        humanRESET = pose(MConstants.humanPlayerPoseRed);
        goalPose = pose(MConstants.goalPoseRed);
    }
    private Pose pose(Pose redPose) {
        if (alliance == Alliance.RED) {
            return redPose;
        } else {
            return redPose.mirror();
        }
    }

    @Override
    public void init() {
        RobotHardware.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        doPoseMath();

        driveTask = new DriveTask(startPose, follower, goalRESET, humanRESET, alliance);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void start() {
        driveTask.startTeleOp();
    }

    @Override
    public void loop() {
        driveTask.update(gamepad1);
        Pose currentPose = driveTask.getPose();
        telemetryM.update();

//        RobotHardware.frontLeftMotor.setPower(gamepad2.left_stick_x);
//        RobotHardware.backLeftMotor.setPower(gamepad2.left_stick_y);
//        RobotHardware.frontRightMotor.setPower(gamepad2.right_stick_x);
//        RobotHardware.backRightMotor.setPower(gamepad2.right_stick_y);

        // ----- Telemetry -----
        telemetryM.debug("position", driveTask.getPose());
        telemetryM.debug("velocity", driveTask.getVelocity());

        telemetry.addData("Pos of base", "X: %.3f, Y: %.3f, H: %.3f°",
                driveTask.getPose().getX(),
                driveTask.getPose().getY(),
                Math.toDegrees(driveTask.getPose().getHeading())
        );

        telemetry.addData("Alliance", alliance);
        telemetry.addData("Goal Pose", goalPose);
        telemetry.addData("Start Pose", startPose);

        telemetry.addData("Distance to goal", getDistanceToGoal(follower.getPose()));

        telemetry.update();
    }
    public double getDistanceToGoal(Pose currentPose) {
        return Math.hypot(goalPose.getX() - currentPose.getX(), goalPose.getY() - currentPose.getY());
    }
}
