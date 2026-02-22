package org.firstinspires.ftc.teamcode.Tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.ButtonHandler;

@Configurable
@TeleOp (name = "Motor_Tune", group = "LinearOpMode")
public class Motor_Tune extends OpMode {
    private final ButtonHandler outtakeVelocityUp = new ButtonHandler();
    private final ButtonHandler outtakeVelocityDown = new ButtonHandler();

    private final ButtonHandler SHOOTER_P_BU = new ButtonHandler();
    private final ButtonHandler SHOOTER_KV_BU = new ButtonHandler();
    private final ButtonHandler SHOOTER_KS_BU = new ButtonHandler();
    private final ButtonHandler SHOOTER_P_BD = new ButtonHandler();
    private final ButtonHandler SHOOTER_KV_BD = new ButtonHandler();
    private final ButtonHandler SHOOTER_KS_BD = new ButtonHandler();

    private boolean outtakeON = false;
    private long targetVelocity = 500;

    private DcMotorEx motor;

    public static double SHOOTER_P = 0.053; // 0.001;
    public static double SHOOTER_KV = 0.0004; // 1.0 / 3000.0;
    public static double SHOOTER_KS = 0.046; // 0.053;


    @Override
    public void init() {
        RobotHardware.init(hardwareMap);

        motor = RobotHardware.outtakeMotor;

        outtakeVelocityUp.setOnPress(() -> targetVelocity += 50);
        outtakeVelocityDown.setOnPress(() -> targetVelocity -= 50);

        SHOOTER_P_BU.setOnPress(() -> SHOOTER_P += 0.0005); // Proportional Gain
        SHOOTER_KV_BU.setOnPress(() -> SHOOTER_KV += 0.000015); // feed forward
        SHOOTER_KS_BU.setOnPress(() -> SHOOTER_KS += 0.007); // Static Feedforward

        SHOOTER_P_BD.setOnPress(() -> SHOOTER_P -= 0.0005);
        SHOOTER_KV_BD.setOnPress(() -> SHOOTER_KV -= 0.000015);
        SHOOTER_KS_BD.setOnPress(() -> SHOOTER_KS -= 0.007);
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        outtakeVelocityUp.update(gamepad1.dpad_up);
        outtakeVelocityDown.update(gamepad1.dpad_down);

        SHOOTER_P_BU.update(gamepad1.triangleWasPressed());
        SHOOTER_P_BD.update(gamepad1.crossWasPressed());

        SHOOTER_KV_BU.update(gamepad1.circleWasPressed());
        SHOOTER_KV_BD.update(gamepad1.squareWasPressed());

        SHOOTER_KS_BU.update(gamepad1.rightBumperWasPressed());
        SHOOTER_KS_BD.update(gamepad1.leftBumperWasPressed());

        motor.setPower(getShooterPower());

        telemetry.addData("Outtake Active", outtakeON);
        telemetry.addData("(Real) Outtake Speed", motor.getVelocity());
        telemetry.addData("Outtake Speed target", targetVelocity);

        telemetry.addData("SHOOTER_P", SHOOTER_P);
        telemetry.addData("SHOOTER_KV", SHOOTER_KV);
        telemetry.addData("SHOOTER_KS", SHOOTER_KS);
        telemetry.update();
    }
    private double getShooterPower() {
        return (SHOOTER_KV * targetVelocity) + (SHOOTER_P * (targetVelocity - motor.getVelocity())) + SHOOTER_KS;
    }
}