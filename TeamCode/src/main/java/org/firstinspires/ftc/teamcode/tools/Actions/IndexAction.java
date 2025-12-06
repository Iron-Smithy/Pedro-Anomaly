// IndexerAction.java
package org.firstinspires.ftc.teamcode.tools.Actions;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.MConstants;

public class IndexAction {
    private final CRServo wheel0;
    private final CRServo wheel1;
    private double currentPower = 0;

    public IndexAction(HardwareMap hardwareMap) {
        wheel0 = RobotHardware.indexerWheel0;
        wheel1 = RobotHardware.indexerWheel1;
    }

    public boolean runIn() {
        if (currentPower != MConstants.indexerPowerIn) {
            wheel0.setPower(MConstants.indexerPowerIn);
            wheel1.setPower(MConstants.indexerPowerIn*-1);
            currentPower = MConstants.indexerPowerIn;
        }
        return true;
    }

    public boolean runOut() {
        if (currentPower != MConstants.indexerPowerOut) {
            wheel0.setPower(MConstants.indexerPowerOut);
            wheel1.setPower(MConstants.indexerPowerOut*-1);
            currentPower = MConstants.indexerPowerOut;
        }
        return true;
    }

    public boolean stop() {
        wheel0.setPower(0);
        wheel1.setPower(0);
        currentPower = 0;
        return true;
    }
}
