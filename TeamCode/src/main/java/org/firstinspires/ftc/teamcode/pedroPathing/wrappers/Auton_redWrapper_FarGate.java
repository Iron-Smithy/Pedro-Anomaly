package org.firstinspires.ftc.teamcode.pedroPathing.wrappers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Auton.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Auton.AutonB_FarGate_0;

// Simple wrapper for AutonB_FarGate_0 that sets the alliance to RED.
// The alliance variable determines which mirrored autonomous path is used.
// No logic is implemented here beyond selecting the mirrored state.

@Autonomous(name = "Auton Red Far Gate", group = "AAA")
public class Auton_redWrapper_FarGate extends AutonB_FarGate_0 {
    @Override
    public void init() {
        setAlliance(Alliance.RED);
        super.init();
    }
}
