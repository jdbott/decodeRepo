package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Pedro 3 has no {@code isRobotStuck()} / {@code breakFollowing()}; this is our replacement.
 * The robot counts as stuck when it is mid-path but has barely moved or turned for {@code timeoutSec}
 * (pinned against a wall or another robot).
 */
public class StuckDetector {

    private static final double MIN_TURN_RATE_RAD_PER_SEC = 0.2;

    private final Follower follower;
    private final double minSpeedInPerSec;
    private final double timeoutSec;
    private final ElapsedTime slowTimer = new ElapsedTime();
    private boolean slow = false;

    public StuckDetector(Follower follower) {
        this(follower, 2.0, 0.75);
    }

    public StuckDetector(Follower follower, double minSpeedInPerSec, double timeoutSec) {
        this.follower = follower;
        this.minSpeedInPerSec = minSpeedInPerSec;
        this.timeoutSec = timeoutSec;
    }

    /** Call once per loop, after {@code follower.update()}. */
    public boolean isStuck() {
        Velocity velocity = follower.velocity();
        boolean crawling = follower.following()
                && !follower.atParametricEnd()
                && Math.hypot(velocity.vx, velocity.vy) < minSpeedInPerSec
                && Math.abs(velocity.omega) < MIN_TURN_RATE_RAD_PER_SEC;

        if (!crawling) {
            slow = false;
            return false;
        }
        if (!slow) {
            slow = true;
            slowTimer.reset();
        }
        return slowTimer.seconds() >= timeoutSec;
    }

    /**
     * Pedro 2's breakFollowing(): abandons the path and holds the current pose. Holding (rather than
     * going idle) means {@code isBusy()} still clears once the robot settles, so state machines advance.
     */
    public void breakFollowing() {
        slow = false;
        follower.hold(follower.pose());
    }
}
