//package org.firstinspires.ftc.teamcode.tools.Actions;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.HeadingInterpolator;
//import com.pedropathing.paths.Path;
//
//public class AutoScoreTaskEEE {
//    public long startTPS;
//    private Follower follower;
//    private OuttakeAction shooter;
//    private IndexAction indexer;
//    private EjectorAction ejector;
//
//    private Pose targetPose;
//    private double targetTPS;
//
//    private boolean active = false;
//
//    private enum State {
//        WAITING_FOR_FEED_START,
//        EJECTOR_MOVING_UP,
//        WAITING_LAUNCH_DELAY,
//        EJECTOR_MOVING_DOWN,
//        CYCLE_COMPLETE // The state where fireCount is incremented and we restart the sequence
//    }
//    private State currentState = State.WAITING_FOR_FEED_START;
//
//    private long timer = 0;
//    private int fireCount = 0;
//    private final int BALLS_TO_FIRE = 3;
//
//    private final long ballFeedDelay = 500;        // ms between shots
//    private final long servoMoveTime = 300;       // ms ejector stays up
//    private final long outtakeLaunchDelay = 50;  // ms to wait before coming down
//
//    public AutoScoreTaskEEE(Follower follower, OuttakeAction shooter, IndexAction indexer, EjectorAction ejector, Pose targetPose, long setTPS) {
//        this.follower = follower;
//        this.shooter = shooter;
//        this.indexer = indexer;
//        this.ejector = ejector;
//        this.targetPose = targetPose;
//        this.startTPS = setTPS;
//    }
//
//    /** Starts the task */
//    public void start() {
//        follower.followPath(
//                follower.pathBuilder()
//                        .addPath(new Path(new BezierLine(follower::getPose, targetPose)))
//                        .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
//                                follower::getHeading,
//                                targetPose.getHeading(),
//                                0.8
//                        ))
//                        .build()
//        );
//        active = true;
//        fireCount = 0;
//        currentState = State.WAITING_FOR_FEED_START; // Reset state when starting
//        timer = System.currentTimeMillis(); // Initialize timer on start
//    }
//
//    /** Call every loop. Use the current outtakeSpeed dynamically */
//    public void update(long currentOuttakeSpeed) {
//        if (!active) return;
//
//        if (!follower.isBusy()) {
//            // Hold robot at target
//            follower.holdPoint(targetPose);
//
//            // Spin up shooter using current outtakeSpeed
//            shooter.spinUp(currentOuttakeSpeed);
//
//            if (shooter.isAtTargetVelocity()) {
//                 long now = System.currentTimeMillis();
//
//                if (fireCount >= BALLS_TO_FIRE) {
//                    finishShootingSequence();
//                    return;
//                }
//
//                // Keep indexer running continuously while actively feeding
//                indexer.runIn();
//
//                switch (currentState) {
//
//                    case WAITING_FOR_FEED_START:
//                        // Only proceed if the initial delay has passed (or immediately the first time)
//                        if (now - timer >= ballFeedDelay) {
//                            ejector.up();
//                            timer = now;
//                            currentState = State.EJECTOR_MOVING_UP;
//                        }
//                        break;
//
//                    case EJECTOR_MOVING_UP:
//                        // Wait for the servo to physically move up
//                        if (now - timer >= servoMoveTime) {
//                            timer = now;
//                            currentState = State.WAITING_LAUNCH_DELAY;
//                        }
//                        break;
//
//                    case WAITING_LAUNCH_DELAY:
//                        // The ejector is up; wait for the ball to launch
//                        if (now - timer >= outtakeLaunchDelay) {
//                            ejector.down();
//                            timer = now;
//                            currentState = State.EJECTOR_MOVING_DOWN;
//                        }
//                        break;
//
//                    case EJECTOR_MOVING_DOWN:
//                        // Wait for the servo to move back down completely
//                        if (now - timer >= servoMoveTime) {
//                            // Cycle complete. Increment count and reset state for the next ball.
//                            fireCount++;
//                            timer = now; // Reset timer for the *next* ball's initial delay
//                            currentState = State.WAITING_FOR_FEED_START;
//                        }
//                        break;
//                }
//            }
//        }
//    }
//
//    private void finishShootingSequence() {
//        // Finished shooting logic
//        indexer.stop();
//        shooter.stop();
//        active = false;
//        // Check if follower is non-null before calling its method
//        if (follower != null) {
//            follower.startTeleopDrive(); // return control
//        }
//    }
//
//    /** Cancel task mid-flight */
//    public void cancel() {
//        active = false;
//        shooter.stop();
//        indexer.stop();
//        ejector.down();
//        // Check if follower is non-null before calling its method
//        if (follower != null) {
//            follower.startTeleopDrive();
//        }
//    }
//
//    /** Is task currently running */
//    public boolean isActive() {
//        return active;
//    }
//}
