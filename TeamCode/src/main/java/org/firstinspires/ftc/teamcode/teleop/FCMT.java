// Field Centric Mecanum TeleOp
package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.tools.ButtonHandler;
import org.firstinspires.ftc.teamcode.tools.MotorGroup;
import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp
public class FCMT extends LinearOpMode {
    private boolean intakeToggle = false;
    private int intakeDir = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware.init(hardwareMap);

        MotorGroup outtake = new MotorGroup(RobotHardware.outtakeMotor1, RobotHardware.outtakeMotor2);
        ButtonHandler buttonA = new ButtonHandler();
        ButtonHandler buttonB = new ButtonHandler();
        ButtonHandler buttonOption = new ButtonHandler();

        boolean outtakeToggle = false;
        int outtakeDir = -1;

        buttonA.setOnPress(() -> intakeToggle = !intakeToggle);
        buttonA.setOnDoubleClick(() -> intakeDir *= -1);
        buttonB.setOnPress(() -> outtakeToggle = !outtakeToggle);
        buttonB.setOnDoubleClick(() -> outtakeDir *= -1);
        buttonOption.setOnPress(() -> RobotHardware.imu.resetYaw());

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            buttonA.update(gamepad1.a);
            buttonB.update(gamepad1.b);
            buttonOption.update(gamepad1.options);

            double y = -gamepad1.left_stick_y / 2;
            double x = gamepad1.left_stick_x / 2;
            double rx = gamepad1.right_stick_x / 2;

            double botHeading = RobotHardware.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX *= 1.1;  // Strafing compensation

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            double intakePower = (intakeToggle ? 1 : 0) * intakeDir;
            double outtakePower = (outtakeToggle ? 1 : 0) * outtakeDir;

            RobotHardware.frontLeftMotor.setPower(frontLeftPower);
            RobotHardware.backLeftMotor.setPower(backLeftPower);
            RobotHardware.frontRightMotor.setPower(frontRightPower);
            RobotHardware.backRightMotor.setPower(backRightPower);
            RobotHardware.intakeMotor.setPower(intakePower);
            outtake.setPower(outtakePower);
        }
    }
}