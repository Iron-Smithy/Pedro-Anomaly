package org.firstinspires.ftc.teamcode.Tasks;

import android.accessibilityservice.MagnificationConfig;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.AlliancePoseProvider;
import org.firstinspires.ftc.teamcode.pedroPathing.Auton.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.MConstants;

public class DriveTask {

    private final Follower follower;

    private Pose startingPose;
    private AlliancePoseProvider poses;
    private double fieldCentricOffset;

    private boolean slowMode = false;
    private final double slowModeMultiplier = 0.25;

    // Exponential joystick shaping
    private final double k = 2.0;
    private final double expKMinus1 = Math.exp(k) - 1;

    public DriveTask(
            Follower follower,
            AlliancePoseProvider poses,
            Pose startingPose
    ) {
        this.follower = follower;
        this.poses = poses;
        this.startingPose = startingPose;

        follower.setStartingPose(startingPose);
        follower.update();

        fieldCentricOffset = poses.fieldCentricReturn();
    }

    public void startTeleOp() {
        follower.startTeleopDrive();
        fieldCentricOffset = startingPose.getHeading();
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
        double turn = gamepad.right_stick_x * 0.70; // 70% from 100% turn power

        slowMode = gamepad.left_stick_button;

        // ---------------------------
        // Exponential shaping
        // ---------------------------
        double driveY = shapeInput(y); // math that emphasizes small joystick movements as small robot movements
        double driveX = shapeInput(x);

        if (slowMode) { // if slow mode reduce speed by a multiple
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
        double denom = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1); // motors cannot spin faster than 100%, math used to maintain speed ratios between wheels when at max motor power

        RobotHardware.frontLeftMotor.setPower((rotY + rotX + turn) / denom);
        RobotHardware.backLeftMotor.setPower((rotY - rotX + turn) / denom);
        RobotHardware.frontRightMotor.setPower((rotY - rotX - turn) / denom);
        RobotHardware.backRightMotor.setPower((rotY + rotX - turn) / denom);

        // ---------------------------
        // Pose Reset Buttons
        // ---------------------------
        if (gamepad.triangleWasPressed()) {
            follower.setPose(poses.get(MConstants.goalResetPoseRed));
            fieldCentricOffset = poses.fieldCentricReturn();
        }

        if (gamepad.squareWasPressed()) {
            follower.setPose(poses.get(MConstants.humanPlayerResetPoseRed));
            fieldCentricOffset = poses.fieldCentricReturn();
        }

        if (gamepad.crossWasPressed()) {
            follower.setPose(poses.get(MConstants.gateResetRed));
            fieldCentricOffset = poses.fieldCentricReturn();
        }

//        if (gamepad.circleWasPressed()) {
//            follower.setPose(poses.get(MConstants.startPoseRow1Red));
//            fieldCentricOffset = poses.fieldCentricReturn();
//        }

        if (gamepad.shareWasPressed()) { // the button labeled share    options   ps is home
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
