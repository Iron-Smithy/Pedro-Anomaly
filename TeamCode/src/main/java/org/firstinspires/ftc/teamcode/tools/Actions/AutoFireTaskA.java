package org.firstinspires.ftc.teamcode.tools.Actions;

public class AutoFireTaskA {
    private OuttakeAction shooter;
    private IndexAction indexer;
    private IntakeAction intake;
    private EjectorAction ejector;

    private long timer = 0;
    private int fireCount = 0;

    private final int BALLS_TO_FIRE = 3;

    private final long ballFeedDelay = 450;
    private final long servoMoveTime = 300;
    private final long outtakeLaunchDelay = 175;

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
    private long[] targetVel;

    public AutoFireTaskA(OuttakeAction shooter, IndexAction indexer, EjectorAction ejector, IntakeAction intake, long[] targetVel) {
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
        shooter.spinUp(targetVel[fireCount]);
        ejector.down();
        intake.stop();
        indexer.stop();
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
                if (fireCount != 0) {
                    indexer.runIn();
                    intake.runIn();
                }

                if (now - timer >= ballFeedDelay * (fireCount == 0 ? 3.50 : 1)) {
                    ejector.up();
                    intake.stop();
                    indexer.stop();
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
                        shooter.spinUp(targetVel[fireCount]);
                        currentState = State.SPINNING_UP;
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
