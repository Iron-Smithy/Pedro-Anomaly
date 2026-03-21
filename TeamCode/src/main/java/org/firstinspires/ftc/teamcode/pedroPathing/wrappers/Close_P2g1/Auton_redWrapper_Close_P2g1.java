package org.firstinspires.ftc.teamcode.pedroPathing.wrappers.Close_P2g1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Auton.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Auton.AutonB_Close_P2g1_0;

@Autonomous(name = "Auton Red 2 g 1", group = "Close")
public class Auton_redWrapper_Close_P2g1 extends AutonB_Close_P2g1_0 {
    @Override
    public void init() {
        setAlliance(Alliance.RED);
        super.init();
    }
}
