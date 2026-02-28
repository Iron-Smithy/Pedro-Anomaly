package org.firstinspires.ftc.teamcode.pedroPathing.wrappers.Far_P32H;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Auton.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Auton.AutonB_Far_P32H_0;

@Autonomous(name = "Auton Blue Far", group = "Far")
public class Auton_blueWrapper_Far_P32H extends AutonB_Far_P32H_0 {
    @Override
    public void init() {
        setAlliance(Alliance.BLUE);
        super.init();
    }
}
