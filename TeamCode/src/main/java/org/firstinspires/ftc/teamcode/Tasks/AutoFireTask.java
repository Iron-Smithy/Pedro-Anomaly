package org.firstinspires.ftc.teamcode.Tasks;

import org.firstinspires.ftc.teamcode.Actions.EjectorAction;
import org.firstinspires.ftc.teamcode.Actions.IndexAction;
import org.firstinspires.ftc.teamcode.Actions.IntakeAction;
import org.firstinspires.ftc.teamcode.Actions.OuttakeAction;
import org.firstinspires.ftc.teamcode.Sensors.BallSensorArray;

/**
 * Auto-fire state machine: spin up shooter, feed balls, and fire when each ball
 * is at the launch position (sensor-based). Falls back to time-based if sensors unavailable.
 */
public class AutoFireTask {
    private final OuttakeAction shooter;
    private final IndexAction indexer;
    private final IntakeAction intake;
    private final EjectorAction ejector;
    private final BallSensorArray ballSensors;

    private long timer = 0;
    private int fireCount = 0;
    private final int ballsToFire;
    private final long targetVel;

    /** Delay before starting first feed (ms). */
    private final long initialFeedDelayMs;
    /** Max time to wait for ball at launch before firing anyway (ms). */
    private final long feedTimeoutMs;
    /** Min time ejector is up before bringing down (ms). */
    private final long ejectorUpMinMs;
    /** Min time ejector is down before next cycle (ms). */
    private final long ejectorDownMinMs;

    private enum State { // firing logic steps
        SPINNING_UP,
        FEEDING,
        EJECTOR_UP,
        EJECTOR_DOWN,
        COMPLETE
    }

    private State currentState = State.FEEDING;
    private boolean active = false;

    public AutoFireTask(OuttakeAction shooter, IndexAction indexer, EjectorAction ejector,
                        IntakeAction intake, long targetVel) {
        this(shooter, indexer, ejector, intake, null, targetVel, 3);
    }

    public AutoFireTask(OuttakeAction shooter, IndexAction indexer, EjectorAction ejector,
                        IntakeAction intake, BallSensorArray ballSensors, long targetVel) {
        this(shooter, indexer, ejector, intake, ballSensors, targetVel, 3);
    }

    public AutoFireTask(OuttakeAction shooter, IndexAction indexer, EjectorAction ejector,
                        IntakeAction intake, BallSensorArray ballSensors, long targetVel, int ballsToFire) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.ejector = ejector;
        this.intake = intake;
        this.ballSensors = ballSensors;
        this.targetVel = targetVel;
        this.ballsToFire = ballsToFire;
        this.initialFeedDelayMs = 300;
        this.feedTimeoutMs = 500;
        this.ejectorUpMinMs = 300;
        this.ejectorDownMinMs = this.ejectorUpMinMs - 20;
    }

    public void start() {
        active = true;
        fireCount = 0;
        currentState = State.SPINNING_UP;
        shooter.spinUp(targetVel);
    }

    public void update() {
        if (!active) return;

        long now = System.currentTimeMillis(); // get current computer time
        boolean sensorSaysReady = ballSensors != null && ballSensors.isLaunchReady(); // does the ball eject sensor detect a ball?

        switch (currentState) {

            case SPINNING_UP:
                if (shooter.isAtTargetVelocity()) { // wait until meet target velocity
                    timer = now;
                    currentState = State.FEEDING;
                }
                break;

            case FEEDING:
                if (fireCount != 0) { // if is not the first ball index ball into eject position
                    indexer.runIn();
                    intake.runIn();
                }
                boolean waitedLongEnough = (now - timer) >= initialFeedDelayMs;
                boolean ballReady = sensorSaysReady || (ballSensors == null && waitedLongEnough);
                boolean timeout = (now - timer) >= feedTimeoutMs;

                if (ballReady || timeout) { // if there is a ball in position, fire. else wait for one but fire after a set time in case of detection issues
                    // Stop/slow intake and indexer so ejector can push ball into outtake without fighting
                    indexer.runIn();
                    intake.runInAt(0.6);
                    ejector.up();
                    timer = now;
                    currentState = State.EJECTOR_UP;
                }
                break;

            case EJECTOR_UP:
                if (now - timer >= ejectorUpMinMs) { // wait for the ejector to reach the up position
                    ejector.down();
                    timer = now;
                    currentState = State.EJECTOR_DOWN;
                }
                break;

            case EJECTOR_DOWN:
                if (now - timer >= ejectorDownMinMs) { // wait for the ejector to reach the down position
                    fireCount++;
                    if (fireCount >= ballsToFire) { // if shoot all the balls finish,
                        currentState = State.COMPLETE;
                    } else { // else restart firing process
                        timer = now;
                        currentState = State.FEEDING;
                    }
                }
                break;

            case COMPLETE:
//                indexer.runInAt(0.3);
//                intake.runInAt(0.4);
//                ejector.down();
//                active = false;
                cancel();
                break;
        }
    }

    public boolean isActive() {
        return active;
    }

    public void cancel() {
        active = false;
        indexer.runInAt(0.1);
        intake.runInAt(0.4);
        ejector.down();
    }

    public void spinUp(double tps) {
        shooter.spinUp(tps);
    }
}
