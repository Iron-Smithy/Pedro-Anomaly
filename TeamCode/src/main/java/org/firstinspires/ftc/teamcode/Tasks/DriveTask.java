package org.firstinspires.ftc.teamcode.Tasks;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class DriveTask {

    private final Follower follower;

    private Pose startingPose;
    private Pose goalRESET;
    private Pose humanRESET;
    private double fieldCentricOffset;

    private boolean slowMode = false;
    private final double slowModeMultiplier = 0.25;

    // Exponential joystick shaping
    private final double k = 2.0;
    private final double expKMinus1 = Math.exp(k) - 1;

    public DriveTask(Pose startingPose, Follower follower, Pose goalRESET, Pose humanRESET) {
        this.startingPose = startingPose;
        this.goalRESET = goalRESET;
        this.humanRESET = humanRESET;
        this.follower = follower;

        follower.setStartingPose(startingPose != null ? startingPose : new Pose());
        follower.update();

        fieldCentricOffset = follower.getHeading();
    }

    public void startTeleOp() {
        follower.startTeleopDrive();
        fieldCentricOffset = 0;
    }

    public void update(Gamepad gamepad) {

        follower.update();

        Pose currentPose = follower.getPose();
        double heading = currentPose.getHeading() - fieldCentricOffset;

        // ---------------------------
        // Input
        // ---------------------------
        double y = -gamepad.left_stick_y;
        double x = gamepad.left_stick_x;
        double turn = gamepad.right_stick_x / 2;

        slowMode = gamepad.left_stick_button;

        // ---------------------------
        // Exponential shaping
        // ---------------------------
        double driveY = shapeInput(y);
        double driveX = shapeInput(x);

        if (slowMode) {
            driveX *= slowModeMultiplier;
            driveY *= slowModeMultiplier;
            turn *= slowModeMultiplier;
        }

        // ---------------------------
        // Field centric transform
        // ---------------------------
        double rotX = driveX * Math.cos(-heading) - driveY * Math.sin(-heading);
        double rotY = driveX * Math.sin(-heading) + driveY * Math.cos(-heading);

        // ---------------------------
        // Mecanum power normalization
        // ---------------------------
        double denom = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);

        RobotHardware.frontLeftMotor.setPower((rotY + rotX + turn) / denom);
        RobotHardware.backLeftMotor.setPower((rotY - rotX + turn) / denom);
        RobotHardware.frontRightMotor.setPower((rotY - rotX - turn) / denom);
        RobotHardware.backRightMotor.setPower((rotY + rotX - turn) / denom);

        // ---------------------------
        // Pose Reset Buttons
        // ---------------------------
        if (gamepad.psWasPressed()) {
            follower.setPose(goalRESET);
            fieldCentricOffset = 0;
        }

        if (gamepad.optionsWasPressed()) {
            follower.setPose(humanRESET);
            fieldCentricOffset = 0;
        }

        if (gamepad.shareWasPressed()) {
            fieldCentricOffset = follower.getHeading();
        }
    }

    private double shapeInput(double input) {
        return Math.signum(input) *
                (Math.exp(k * Math.abs(input)) - 1) /
                expKMinus1;
    }

    public Pose getPose() {
        return follower.getPose();
    }

    public Vector getVelocity() {
        return follower.getVelocity();
    }

    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    public double getHeading() {
        return follower.getHeading();
    }
}
