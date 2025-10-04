// Field Centric Mecanum TeleOp with Acceleration Smoothing
package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.tools.ButtonHandler;
import org.firstinspires.ftc.teamcode.tools.MotorGroup;
// import org.firstinspires.ftc.teamcode.RobotHardware;
import static org.firstinspires.ftc.teamcode.RobotHardware.*;

@TeleOp
public class FCMTAS extends LinearOpMode {
    private boolean intakeToggle = false;
    private int intakeDir = 1;

    private boolean outtakeToggle = false;
    private int outtakeDir = -1;

    double k = 2.0;
    double expKMinus1 = Math.exp(k) - 1; // Precompute at init
    // double j = Math.signum(x) * (Math.exp(k * Math.abs(x)) - 1) / expKMinus1;

    public double smooth(double current, double target, double step) {
        if (target > current) {
            current += step;
            if (current > target) current = target;
        } else if (target < current) {
            current -= step;
            if (current < target) current = target;
        }
        return current;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        init(hardwareMap);

        MotorGroup outtakeMotors = new MotorGroup(outtakeMotor1, outtakeMotor2);
        MotorGroup driveMotors = new MotorGroup(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);

        ButtonHandler buttonA = new ButtonHandler();
        ButtonHandler buttonB = new ButtonHandler();
        ButtonHandler buttonOption = new ButtonHandler();

        double vx = 0.0;
        double vy = 0.0;
        double vr = 0.0;

        buttonA.setOnPress(() -> intakeToggle = !intakeToggle);
        buttonA.setOnDoubleClick(() -> intakeDir *= -1);
        buttonB.setOnPress(() -> outtakeToggle = !outtakeToggle);
        buttonB.setOnDoubleClick(() -> outtakeDir *= -1);
        buttonOption.setOnPress(() -> imu.resetYaw());

        driveMotors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            buttonA.update(gamepad1.a);
            buttonB.update(gamepad1.b);
            buttonOption.update(gamepad1.options);

            // obtain the joystick values
            double jy = -gamepad1.left_stick_y;
            double jx = gamepad1.left_stick_x;
            double jrx = gamepad1.right_stick_x;
            // Make small motor movments make up a greater portion of the joystick value range to make small movments easier
            // robot target speed value
            double yt = Math.signum(jy) * (Math.exp(k * Math.abs(jy)) - 1) / expKMinus1;
            double xt = Math.signum(jx) * (Math.exp(k * Math.abs(jx)) - 1) / expKMinus1;
            double rxt = Math.signum(jrx) * (Math.exp(k * Math.abs(jrx)) - 1) / expKMinus1;

            double step = 0.01; // how fast to ramp per loop                    tune ME!!!

            vx = smooth(vx, xt, step);   // strafe smoothing
            vy = smooth(vy, yt, step);   // forward/back smoothing
            vr = smooth(vr, rxt, step);  // rotation smoothing


            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = vx * Math.cos(-botHeading) - vy * Math.sin(-botHeading);
            double rotY = vx * Math.sin(-botHeading) + vy * Math.cos(-botHeading);

            rotX *= 1.1;  // Strafing compensation

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(vr), 1);
            double frontLeftPower  = (rotY + rotX + vr) / denominator;
            double backLeftPower   = (rotY - rotX + vr) / denominator;
            double frontRightPower = (rotY - rotX - vr) / denominator;
            double backRightPower  = (rotY + rotX - vr) / denominator;

            double intakePower = (intakeToggle ? 1 : 0) * intakeDir;
            double outtakePower = (outtakeToggle ? 1 : 0) * outtakeDir;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            intakeMotor.setPower(intakePower);
            outtakeMotors.setPower(outtakePower);
        }
    }
}