package org.firstinspires.ftc.teamcode.tools.Actions;

import com.pedropathing.util.Timer;

public class AutoFireTask {
    private OuttakeAction shooter;
    private IndexAction indexer;
    private IntakeAction intake;
    private EjectorAction ejector;

    private long timer = 0;
    private int fireCount = 0;

    private final int BALLS_TO_FIRE = 3;

    private final long ballFeedDelay = 400;
    private final long servoMoveTime = 300;
    private final long outtakeLaunchDelay = 150;

    private enum State {
        SPINNING_UP,
        WAITING_FOR_FEED_START,
        EJECTOR_MOVING_UP,
        WAITING_LAUNCH_DELAY,
        EJECTOR_MOVING_DOWN,
        COMPLETE
    }

    private State currentState = State.WAITING_FOR_FEED_START;
    private boolean active = false;
    private long targetVel;

    public AutoFireTask(OuttakeAction shooter, IndexAction indexer, EjectorAction ejector, IntakeAction intake, long targetVel) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.intake = intake;
        this.ejector = ejector;
        this.targetVel = targetVel;
    }

    public void start() {
        active = true;
        fireCount = 0;
        currentState = State.SPINNING_UP;
        shooter.spinUp(targetVel);
    }

    public void update() {
        if (!active) return;

        long now = System.currentTimeMillis();

        switch (currentState) {

            case SPINNING_UP:
                if (shooter.isAtTargetVelocity()) {
                    timer = now;
                    currentState = State.WAITING_FOR_FEED_START;
                }
                break;

            case WAITING_FOR_FEED_START:
                indexer.runIn();
                intake.runIn();

                if (now - timer >= ballFeedDelay) {
                    ejector.up();
                    timer = now;
                    currentState = State.EJECTOR_MOVING_UP;
                }
                break;

            case EJECTOR_MOVING_UP:
                if (now - timer >= servoMoveTime) {
                    timer = now;
                    currentState = State.WAITING_LAUNCH_DELAY;
                }
                break;

            case WAITING_LAUNCH_DELAY:
                if (now - timer >= outtakeLaunchDelay) {
                    ejector.down();
                    timer = now;
                    currentState = State.EJECTOR_MOVING_DOWN;
                }
                break;

            case EJECTOR_MOVING_DOWN:
                if (now - timer >= servoMoveTime) {
                    fireCount++;
                    if (fireCount >= BALLS_TO_FIRE) {
                        currentState = State.COMPLETE;
                    } else {
                        timer = now;
                        currentState = State.WAITING_FOR_FEED_START;
                    }
                }
                break;

            case COMPLETE:
//                shooter.stop();
                indexer.stop();
                intake.stop();
                ejector.down();
                active = false;
                break;
        }
    }

    public boolean isActive() { return active; }

    public void cancel() {
        active = false;
//        shooter.stop();
        indexer.stop();
        intake.stop();
        ejector.down();
    }
}
