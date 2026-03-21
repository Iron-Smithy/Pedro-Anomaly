package org.firstinspires.ftc.teamcode.pedroPathing;
import com.pedropathing.follower.Follower;
public class RobotUtils {
    /**
     * Checks if the robot is finished with its path and physically still.
     * Call this in your state machine: if (RobotUtils.isStable(follower)) {...}
     */
    public static boolean isStable(Follower follower) {
        //1. Pedro says it's done (or timed out)
        boolean pathFinished = !follower.isBusy();

        //2. Velocity is near zero (adjust 0.5 to be more/less strict)
        boolean isStill = follower.getVelocity().getMagnitude() < 0.5;

        //3. Heading is within ~1.5 degrees
        boolean headingLocked = Math.abs(follower.getHeadingError()) < Math.toRadians(1.5);

        return pathFinished && isStill && headingLocked;
    }
}
