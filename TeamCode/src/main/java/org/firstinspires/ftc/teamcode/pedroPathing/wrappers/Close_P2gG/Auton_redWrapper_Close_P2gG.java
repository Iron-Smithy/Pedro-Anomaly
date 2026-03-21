package org.firstinspires.ftc.teamcode.pedroPathing.wrappers.Close_P2gG;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Auton.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Auton.AutonB_Close_P2gG_0;

@Autonomous(name = "Auton Red 2 g G", group = "Close")
public class Auton_redWrapper_Close_P2gG extends AutonB_Close_P2gG_0 {
    @Override
    public void init() {
        setAlliance(Alliance.RED);
        super.init();
    }
}