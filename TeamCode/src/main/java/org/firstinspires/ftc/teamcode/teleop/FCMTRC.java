// Field Centric Mecanum TeleOp with and Rotation Clamping
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
public class FCMTRC extends LinearOpMode {
    private boolean intakeToggle = false;
    private int intakeDir = 1;

    private boolean outtakeToggle = false;
    private int outtakeDir = -1;

    double k = 2.0;
    double expKMinus1 = Math.exp(k) - 1; // Precompute at init;

    @Override
    public void runOpMode() throws InterruptedException {
        init(hardwareMap);

        MotorGroup outtakeMotors = new MotorGroup(outtakeMotor1, outtakeMotor2);
        MotorGroup driveMotors = new MotorGroup(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);

        ButtonHandler buttonA = new ButtonHandler();
        ButtonHandler buttonB = new ButtonHandler();
        ButtonHandler buttonOption = new ButtonHandler();

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

            // 1. Read joystick
            double jy = -gamepad1.left_stick_y;
            double jx = gamepad1.left_stick_x;
            double jrx = gamepad1.right_stick_x;

            // 2. Exponential stick curve
            double yt = Math.signum(jy) * (Math.exp(k * Math.abs(jy)) - 1) / expKMinus1;
            double xt = Math.signum(jx) * (Math.exp(k * Math.abs(jx)) - 1) / expKMinus1;

            // 3. Heading hold logic
            double actualHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rxt;

            if (Math.abs(jrx) > 0.05) {
                // Manual rotation
                botHeadingTarget = actualHeading;
                rxt = jrx;
            } else {
                // Auto heading hold
                double headingError = botHeadingTarget - actualHeading;
                headingError = Math.atan2(Math.sin(headingError), Math.cos(headingError));

                double kP = 2.0; // tune ME!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                double deadband = Math.toRadians(1.5);
                double correction = 0.0;

                if (Math.abs(headingError) > deadband)
                    correction = headingError * kP;

                rxt = correction;
            }

            // 4. Apply translation directly
            double vx = xt;
            double vy = yt;
            double vr = rxt;

            // 5. Field-centric drive transform
            double rotX = vx * Math.cos(-actualHeading) - vy * Math.sin(-actualHeading);
            double rotY = vx * Math.sin(-actualHeading) + vy * Math.cos(-actualHeading);
            rotX *= 1.1; // strafe compensation

            // 6. Motor powers
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