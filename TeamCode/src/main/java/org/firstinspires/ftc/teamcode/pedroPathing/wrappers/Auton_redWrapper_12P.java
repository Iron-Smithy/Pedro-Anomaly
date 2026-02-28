package org.firstinspires.ftc.teamcode.pedroPathing.wrappers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Auton.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Auton.AutonB_12P_0;

// Simple wrapper for AutonB_12P_0 that sets the alliance to RED.
// The alliance variable determines which mirrored autonomous path is used.
// No logic is implemented here beyond selecting the mirrored state.

@Autonomous(name = "Auton Red 12P", group = "AAA")
public class Auton_redWrapper_12P extends AutonB_12P_0 {
    @Override
    public void init() {
        setAlliance(Alliance.RED);
        super.init();
    }
}
