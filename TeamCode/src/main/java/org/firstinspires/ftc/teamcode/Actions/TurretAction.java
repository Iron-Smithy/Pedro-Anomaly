// OuttakeAction.java
package org.firstinspires.ftc.teamcode.Actions;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.MConstants;

public class TurretAction {
    public final DcMotor motor;

    private final int leftTickMax = -265; // range = 0 < 288 > 577
    private final int rightTickMax = 265; // right most based on 3d printed limits
    private final double leftRadMax = -Math.PI / 2;
    private final double rightRadMax = Math.PI / 2;
    public final int center = (leftTickMax + rightTickMax) / 2;
    public PIDFController controller = new PIDFController(MConstants.turretPIDFCoefficent);

    public TurretAction(HardwareMap hardwareMap) {
        motor = RobotHardware.turret;
        motor.setDirection(DcMotor.Direction.REVERSE);
//        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); Don't reset on initialization of action
    }

    public void updatePIDF(com.pedropathing.control.PIDFCoefficients coefficients) {
        this.controller.setCoefficients(coefficients);
        this.controller.reset();
    }
    public void runToTick(int tick) {
        int target = limitTickValues(tick);

        // Update the internal target and current position
        controller.setTargetPosition(target);
        controller.updatePosition(motor.getCurrentPosition());

        // Execute the PIDF math to get the power value
        // run() returns the double you need for the motor
        double power = controller.run();

        // Apply to the motor
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(power);

//        motor.setTargetPosition(target);
//        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motor.setPower(1);
    }

    public boolean stop() {
        motor.setPower(0);
        return true;
    }

    public int limitTickValues(int tick) {
        return Math.max(leftTickMax, Math.min(rightTickMax, tick));
    }
    public double limitRadValues(double Angle) {
        return Math.max(leftRadMax, Math.min(rightRadMax, Angle));
    }
    public int radToTick(double angle) {
        return (int) (angle * MConstants.MAGIC);
    }

    public double calculateWrapAngles(double targetRad) {
        double wrapped = Math.atan2(Math.sin(targetRad), Math.cos(targetRad));

        if (wrapped > rightRadMax) return rightRadMax;
        if (wrapped < leftRadMax) return leftRadMax;

        return wrapped;
    }
}
