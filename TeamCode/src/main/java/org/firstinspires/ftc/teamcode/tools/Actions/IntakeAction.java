// IntakeAction.java
package org.firstinspires.ftc.teamcode.tools.Actions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.MConstants;

public class IntakeAction {
    private final DcMotor intakeMotor;

    public IntakeAction(HardwareMap hardwareMap) {
        intakeMotor = RobotHardware.intakeMotor;
    }

    public void runIn() {
        spin(MConstants.intakePowerIn);
    }

    public void runInAt(double pow) {
        spin(MConstants.intakePowerIn * pow);
    }

    public void runOut() {
        spin(MConstants.intakePowerOut);
    }

    public void stop() {
        spin(0);
    }

    public void spin(double power) {
        intakeMotor.setPower(power);
    }
}