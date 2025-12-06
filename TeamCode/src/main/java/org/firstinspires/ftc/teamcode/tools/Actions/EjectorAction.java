// EjectorAction.java
package org.firstinspires.ftc.teamcode.tools.Actions;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.MConstants;

public class EjectorAction {
    public final Servo paddle;
    private boolean isInit = false;
    private boolean isUp = false;

    public EjectorAction(HardwareMap hardwareMap) {
        paddle = RobotHardware.indexerPaddle;
    }

    public boolean run() { // dont work
        if (!isInit) {
            paddle.setPosition(MConstants.ejectorUp);
            isInit = true;
            isUp = true;
            return false; // still running
        }
        if (isUp) {
            // automatically move back down next loop
            paddle.setPosition(MConstants.ejectorDown);
            isUp = false;
            return false; // still running
        }
        if (!isUp && paddle.getPosition() >= MConstants.ejectorDown + 0.1) {
            return false;
        }
        return true; // finished
    }
    public void up() {
        paddle.setPosition(MConstants.ejectorUp);
    }
    public void down() {
        paddle.setPosition(MConstants.ejectorDown);
    }
    public void reset() {
        isInit = false;
        isUp = false;
        paddle.setPosition(MConstants.ejectorDown);
    }
}
