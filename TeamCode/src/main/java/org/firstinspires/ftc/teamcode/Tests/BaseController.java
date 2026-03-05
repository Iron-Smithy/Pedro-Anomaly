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
import org.firstinspires.ftc.teamcode.pedroPathing.AlliancePoseProvider;
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
@TeleOp(name = "Base Controller", group = "Tests")
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
    private Pose goalPose;
    AlliancePoseProvider poses;

    @Override
    public void init() {
        RobotHardware.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        poses = new AlliancePoseProvider(alliance);

        startPose = poses.get(new Pose(96,111,Math.toRadians(45)));

        driveTask = new DriveTask(
                follower,
                poses,
                startPose
        );

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
