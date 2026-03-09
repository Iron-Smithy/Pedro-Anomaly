package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Auton.Alliance;

public class AlliancePoseProvider {

    private Alliance alliance;

    public AlliancePoseProvider(Alliance alliance) {
        this.alliance = alliance;
    }

    public Pose get(Pose redPose) {
        return alliance == Alliance.RED ? redPose : redPose.mirror();
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public boolean isRed() {
        return alliance == Alliance.RED;
    }

    public double fieldCentricReturn() {
        return alliance == Alliance.RED ? 0 : Math.toRadians(180);
    }
}