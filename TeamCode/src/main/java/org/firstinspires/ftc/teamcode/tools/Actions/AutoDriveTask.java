package org.firstinspires.ftc.teamcode.tools.Actions;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;

public class AutoDriveTask {
    private final Follower follower;
    private final Pose targetPose;

    private boolean active = false;
    private boolean holding = false;

    public AutoDriveTask(Follower follower, Pose targetPose) {
        this.follower = follower;
        this.targetPose = targetPose;
    }

    public void start() {
        follower.followPath(
                follower.pathBuilder()
                        .addPath(new Path(new BezierLine(follower::getPose, targetPose)))
                        .setHeadingInterpolation(
                                HeadingInterpolator.linearFromPoint(
                                        follower::getHeading,
                                        targetPose.getHeading(),
                                        0.8
                                )
                        )
                        .build()
        );

        active = true;
        holding = false;
    }

    public void update() {
        if (!active) return;

        // Path finished → enter HOLD
        if (!holding && !follower.isBusy()) {
            follower.holdPoint(targetPose);
            holding = true;
        }
    }

    public void cancel() {
        active = false;
        holding = false;

        // THIS breaks the hold controller
        follower.startTeleopDrive(true);
    }

    public boolean isActive() {
        return active;
    }
}