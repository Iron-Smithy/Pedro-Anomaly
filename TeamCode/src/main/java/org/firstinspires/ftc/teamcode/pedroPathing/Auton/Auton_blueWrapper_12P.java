package org.firstinspires.ftc.teamcode.pedroPathing.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auton Blue 12P", group = "AAA")
public class Auton_blueWrapper_12P extends AutonB_12P_0 {
    @Override
    public void init() {
        setAlliance(Alliance.BLUE);
        super.init();
    }
}
