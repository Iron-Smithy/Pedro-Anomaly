package org.firstinspires.ftc.teamcode.tools.Actions;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

public class AutoDriveTaskEX {
    private final Follower follower;
    private final Pose targetPose;
    private boolean active = false;

    public AutoDriveTaskEX(Follower follower, Pose targetPose) {
        this.follower = follower;
        this.targetPose = targetPose;
    }

    /**
     * Start the drive. Instead of a one-time path, we use holdPoint
     * so it constantly recalculates power to stay on the target.
     */
    public void start() {
        if (!active) {
            follower.holdPoint(targetPose);
            active = true;
        }
    }

    /**
     * Call this in the loop. It ensures the follower is still
     * focused on the holdPoint.
     */
    public void update() {
        if (!active) return;
        
        // Ensure the follower hasn't switched back to teleop 
        // while the task is supposed to be active.
        if (!follower.isBusy()) {
            follower.holdPoint(targetPose);
        }
    }

    /**
     * Critical: This hands control back to the joysticks.
     */
    public void cancel() {
        active = false;
        follower.startTeleopDrive();
    }

    public boolean isActive() {
        return active;
    }
}