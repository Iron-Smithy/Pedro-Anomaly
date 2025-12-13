//package org.firstinspires.ftc.teamcode.tools.Actions;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.HeadingInterpolator;
//import com.pedropathing.paths.Path;
//import com.pedropathing.paths.PathChain;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.RobotHardware;
//import org.firstinspires.ftc.teamcode.pedroPathing.MConstants;
//import org.firstinspires.ftc.teamcode.tools.Actions.*;
//import org.firstinspires.ftc.teamcode.tools.ButtonHandler;
//
//import java.util.function.Supplier;
//
//public class AutoScoreTaskE {
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
//    private int fireCount = 0;
//    private long lastFireTime = 0;
//
//    private final long feedDelay = 1300;      // ms ejector stays up
//    private final long interShotDelay = 15000; // ms between shots
//
//    public AutoScoreTaskE(Follower follower, OuttakeAction shooter, IndexAction indexer, EjectorAction ejector, Pose targetPose, long setTPS) {
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
//                long now = System.currentTimeMillis();
//
//                // Feed 3 balls
//                if (fireCount < 3) {
//                    if (now - lastFireTime >= interShotDelay) {
//                        ejector.up(); // launch
//                        lastFireTime = now;
//                        fireCount++;
//                    } else if (now - lastFireTime >= feedDelay) {
//                        ejector.down(); // ready next
//                    }
//                    indexer.runIn();
//                } else {
//                    // Finished shooting
//                    indexer.stop();
//                    shooter.stop();
//                    active = false;
//                    follower.startTeleopDrive(); // return control
//                }
//            }
//        }
//    }
//
//    /** Cancel task mid-flight */
//    public void cancel() {
//        active = false;
//        shooter.stop();
//        indexer.stop();
//        ejector.down();
//        follower.startTeleopDrive();
//    }
//
//    /** Is task currently running */
//    public boolean isActive() {
//        return active;
//    }
//}
