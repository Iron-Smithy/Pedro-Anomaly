package org.firstinspires.ftc.teamcode.pedroPathing.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auton Red 12P", group = "AAA")
public class Auton_redWrapper_12P extends AutonB_12P_0 {
    @Override
    public void init() {
        setAlliance(Alliance.RED);
        super.init();
    }
}
