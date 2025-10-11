// Field Centric Mecanum TeleOp
package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.tools.ButtonHandler;
import org.firstinspires.ftc.teamcode.tools.MotorGroup;
import static org.firstinspires.ftc.teamcode.RobotHardware.*;

@TeleOp
public class FCMT extends LinearOpMode {
    private boolean intakeToggle = false;
    private int intakeDir = -1;

    private boolean outtakeToggle = false;
    private final int outtakeDir = 1;

    double k = 2.0;
    double expKMinus1 = Math.exp(k) - 1; // Precompute at init;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware.init(hardwareMap);

        MotorGroup outtakeMotors = new MotorGroup(outtakeMotor1, outtakeMotor2);
        MotorGroup driveMotors = new MotorGroup(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);

        ButtonHandler leftPaddle = new ButtonHandler();
        ButtonHandler rightPaddle = new ButtonHandler();
        ButtonHandler buttonX = new ButtonHandler();
        ButtonHandler buttonA = new ButtonHandler();
        ButtonHandler buttonOption = new ButtonHandler();

        imu.resetYaw();

        // Intake
        buttonA.setOnPress(() -> intakeToggle = !intakeToggle);
        leftPaddle.setOnPress(() -> intakeDir = 1);
        rightPaddle.setOnPress(() -> intakeDir = -1);

        // Outtake
        buttonX.setOnPress(() -> outtakeToggle = !outtakeToggle);

        // Imu
        buttonOption.setOnPress(() -> imu.resetYaw());

        driveMotors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            buttonA.update(gamepad1.a);
            leftPaddle.update(gamepad1.left_bumper);
            rightPaddle.update(gamepad1.right_bumper);

            buttonX.update(gamepad1.x);
            buttonOption.update(gamepad1.options);

            // 1. Read joystick
            double jy = -gamepad1.left_stick_y;
            double jx = gamepad1.left_stick_x;
            double jrx = gamepad1.right_stick_x;

            // 2. Exponential stick curve
            double y = Math.signum(jy) * (Math.exp(k * Math.abs(jy)) - 1) / expKMinus1;
            double x = Math.signum(jx) * (Math.exp(k * Math.abs(jx)) - 1) / expKMinus1;

            double botHeading = RobotHardware.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // 5. Field-centric drive transform
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX *= 1.1;  // Strafing compensation

            // 6. Motor powers
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(jrx), 1);
            frontLeftMotor.setPower((rotY + rotX + jrx) / denominator);
            backLeftMotor.setPower((rotY - rotX + jrx) / denominator);
            frontRightMotor.setPower((rotY - rotX - jrx) / denominator);
            backRightMotor.setPower((rotY + rotX - jrx) / denominator);

            double intakePower = (intakeToggle ? 1 : 0) * intakeDir;
            double outtakePower = (outtakeToggle ? 1 : 0) * outtakeDir;

            intakeMotor.setPower(intakePower);
            outtakeMotors.setPower(outtakePower);
        }
    }
}