// Field Centric Mecanum TeleOp with Acceleration Smoothing and Rotation Clamping
// tha almighty “Fic-mt-a-sirc” (fick-MIT-uh-sirk)
package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.tools.ButtonHandler;
import org.firstinspires.ftc.teamcode.tools.MotorGroup;
import static org.firstinspires.ftc.teamcode.RobotHardware.*;

@TeleOp
public class FCMTASRC extends LinearOpMode {
    private boolean intakeToggle = false;
    private int intakeDir = 1;

    private boolean outtakeToggle = false;
    private int outtakeDir = -1;

    double k = 2.0;
    double expKMinus1 = Math.exp(k) - 1; // Precompute at init;

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

        double step = 0.01; // how fast to increase per loop                    tune ME!!!
        double kPHeading = 2.0;                                          // tune ME!!!

        imu.resetYaw();
        double botHeadingTarget = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

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

            double actualBotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // 1. Read joystick
            double jy = -gamepad1.left_stick_y;
            double jx = gamepad1.left_stick_x;
            double jrx = gamepad1.right_stick_x;

            // 2. Compute target velocities (exponential curve)
            double yt = Math.signum(jy) * (Math.exp(k * Math.abs(jy)) - 1) / expKMinus1;
            double xt = Math.signum(jx) * (Math.exp(k * Math.abs(jx)) - 1) / expKMinus1;
            double rxt = Math.signum(jrx) * (Math.exp(k * Math.abs(jrx)) - 1) / expKMinus1;

            // 3. Heading correction
            double headingError = botHeadingTarget - actualBotHeading;
            headingError = Math.atan2(Math.sin(headingError), Math.cos(headingError));
            double correction = (Math.abs(jrx) < 0.05) ? headingError * kPHeading : 0;
            if (Math.abs(jrx) >= 0.05) botHeadingTarget = actualBotHeading;
            rxt += correction;

            // 4. Smooth final commands
            vx = smooth(vx, xt, step);
            vy = smooth(vy, yt, step);
            vr = smooth(vr, rxt, step);

            // 5. Field-centric transform
            double rotX = vx * Math.cos(-actualBotHeading) - vy * Math.sin(-actualBotHeading);
            double rotY = vx * Math.sin(-actualBotHeading) + vy * Math.cos(-actualBotHeading);
            rotX *= 1.1;  // strafing compensation  TUNE ME!!!

            // 6. Motor power calculation & set
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(vr), 1);
            frontLeftMotor.setPower((rotY + rotX + vr) / denominator);
            backLeftMotor.setPower((rotY - rotX + vr) / denominator);
            frontRightMotor.setPower((rotY - rotX - vr) / denominator);
            backRightMotor.setPower((rotY + rotX - vr) / denominator);


            double intakePower = (intakeToggle ? 1 : 0) * intakeDir;
            double outtakePower = (outtakeToggle ? 1 : 0) * outtakeDir;

            intakeMotor.setPower(intakePower);
            outtakeMotors.setPower(outtakePower);
        }
    }
}