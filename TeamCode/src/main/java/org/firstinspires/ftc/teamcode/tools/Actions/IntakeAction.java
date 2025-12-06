// IntakeAction.java
package org.firstinspires.ftc.teamcode.tools.Actions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.MConstants;

public class IntakeAction {
    private final DcMotor intakeMotor;
    private double currentPower = 0;

    public IntakeAction(HardwareMap hardwareMap) {
        intakeMotor = RobotHardware.intakeMotor;
    }

    public boolean runIn() {
        if (currentPower != MConstants.intakePowerIn) {
            intakeMotor.setPower(MConstants.intakePowerIn);
            currentPower = MConstants.intakePowerIn;
        }
        return true; // always "ready" because motor just spins
    }

    public boolean runOut() {
        if (currentPower != MConstants.intakePowerOut) {
            intakeMotor.setPower(MConstants.intakePowerOut);
            currentPower = MConstants.intakePowerOut;
        }
        return true;
    }

    public boolean stop() {
        intakeMotor.setPower(0);
        currentPower = 0;
        return true;
    }
}
