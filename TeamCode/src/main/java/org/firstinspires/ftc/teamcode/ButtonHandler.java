package org.firstinspires.ftc.teamcode;

public class ButtonHandler {
    private boolean lastState = false;
    private boolean currentState = false;
    private long lastPressTime = 0;
    private long lastReleaseTime = 0;
    private long doubleClickThreshold = 300;

    private boolean doubleClicked = false;
    private boolean pressedThisFrame = false;
    private boolean releasedThisFrame = false;

    private Runnable onPress;
    private Runnable onHold;
    private Runnable onRelease;
    private Runnable onDoubleClick;

    public void setOnPress(Runnable action) { this.onPress = action; }
    public void setOnHold(Runnable action) { this.onHold = action; }
    public void setOnRelease(Runnable action) { this.onRelease = action; }
    public void setOnDoubleClick(Runnable action) { this.onDoubleClick = action; }

    public void update(boolean isPressed) {
        currentState = isPressed;
        doubleClicked = false;
        pressedThisFrame = false;
        releasedThisFrame = false;

        long currentTime = System.currentTimeMillis();

        if (!lastState && currentState) {
            pressedThisFrame = true;
            if (currentTime - lastReleaseTime <= doubleClickThreshold) {
                doubleClicked = true;
                if (onDoubleClick != null) onDoubleClick.run();
            }
            if (onPress != null) onPress.run();
            lastPressTime = currentTime;
        }

        if (lastState && currentState && onHold != null) onHold.run();

        if (lastState && !currentState) {
            releasedThisFrame = true;
            if (onRelease != null) onRelease.run();
            lastReleaseTime = currentTime;
        }

        lastState = currentState;
    }
    public void update(float howMuchPressed, float whenConsideredPressed) {
        currentState = howMuchPressed >= whenConsideredPressed;
        doubleClicked = false;
        pressedThisFrame = false;
        releasedThisFrame = false;

        long currentTime = System.currentTimeMillis();

        if (!lastState && currentState) {
            pressedThisFrame = true;
            if (currentTime - lastReleaseTime <= doubleClickThreshold) {
                doubleClicked = true;
                if (onDoubleClick != null) onDoubleClick.run();
            }
            if (onPress != null) onPress.run();
            lastPressTime = currentTime;
        }

        if (lastState && currentState && onHold != null) onHold.run();

        if (lastState && !currentState) {
            releasedThisFrame = true;
            if (onRelease != null) onRelease.run();
            lastReleaseTime = currentTime;
        }

        lastState = currentState;
    }

    public boolean wasPressed() { return pressedThisFrame; }
    public boolean isPressed() { return currentState; }
    public boolean isHeld() { return lastState && currentState; }
    public boolean wasReleased() { return releasedThisFrame; }
    public boolean isReleased() { return !currentState; }
    public boolean isDoubleClicked() { return doubleClicked; }
}
