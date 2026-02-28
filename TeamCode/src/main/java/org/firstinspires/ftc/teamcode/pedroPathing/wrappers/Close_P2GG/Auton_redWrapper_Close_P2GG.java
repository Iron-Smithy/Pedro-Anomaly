package org.firstinspires.ftc.teamcode.pedroPathing.wrappers.Close_P2GG;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Auton.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Auton.AutonB_Close_P2GG_0;

@Autonomous(name = "Auton Red Gate Cycle", group = "Close")
public class Auton_redWrapper_Close_P2GG extends AutonB_Close_P2GG_0 {
    @Override
    public void init() {
        setAlliance(Alliance.RED);
        super.init();
    }
}