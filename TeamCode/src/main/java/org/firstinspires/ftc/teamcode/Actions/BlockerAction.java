// EjectorAction.java
package org.firstinspires.ftc.teamcode.Actions;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.MConstants;

public class BlockerAction {
    public final Servo paddle;
    private boolean isInit = false;
    private boolean isOut = false;

    public BlockerAction(HardwareMap hardwareMap) {
        paddle = RobotHardware.blocker;
    }
    public void in() {
        paddle.setPosition(MConstants.blockerIn);
    }
    public void out() {
        paddle.setPosition(MConstants.blockerOut);
    }
    public void reset() {
        isInit = false;
        isOut = false;
        paddle.setPosition(MConstants.blockerIn);
    }
}
