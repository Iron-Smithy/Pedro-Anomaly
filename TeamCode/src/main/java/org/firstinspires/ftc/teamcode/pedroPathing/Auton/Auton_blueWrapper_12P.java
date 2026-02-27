package org.firstinspires.ftc.teamcode.pedroPathing.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// Simple wrapper for AutonB_12P_0 that sets the alliance to BLUE.
// The alliance variable determines which mirrored autonomous path is used.
// No logic is implemented here beyond selecting the mirrored state.

@Autonomous(name = "Auton Blue 12P", group = "AAA")
public class Auton_blueWrapper_12P extends AutonB_12P_0 { // a wrapper
    @Override
    public void init() {
        setAlliance(Alliance.BLUE);
        super.init();
    }
}
