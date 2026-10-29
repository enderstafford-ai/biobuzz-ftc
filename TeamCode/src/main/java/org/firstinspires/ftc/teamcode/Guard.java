package org.firstinspires.ftc.teamcode;

/*
 * Keeps one mechanism (intake, brake, flywheel) from ever hurting the drive.
 *
 * Everything that talks to that mechanism's hub goes through run(). If a call throws, or takes so long
 * that the hub is clearly not answering (Expansion Hub cable knocked out mid-match), the mechanism is
 * switched off for now and the robot keeps driving at full speed. It's tried again only when the driver
 * asks for that mechanism, at most every 2 seconds, so it comes back by itself if the cable is pushed back in.
 */
public class Guard {

    private static final long SLOW_NS = 80_000_000L;       // a normal hub call takes 2-3 ms
    private static final int STRIKES = 2;                   // slow calls in a row before giving up
    private static final long RETRY_NS = 2_000_000_000L;

    private final String what;
    private boolean down;
    private int slowCalls;
    private long downSinceNs;
    private String problem = "";

    public Guard(String what) {
        this.what = what;
    }

    /** True if the mechanism is working (or it's time to try it again because the driver wants it). */
    public boolean ready(boolean driverWantsIt) {
        if (!down) return true;
        return driverWantsIt && System.nanoTime() - downSinceNs > RETRY_NS;
    }

    /** Runs one bit of hub talk. Returns false if it failed (the mechanism is then off for now). */
    public boolean run(Runnable hubTalk) {
        long started = System.nanoTime();
        try {
            hubTalk.run();
        } catch (RuntimeException e) {
            fail("stopped answering");
            return false;
        }
        if (System.nanoTime() - started > SLOW_NS) {
            if (++slowCalls >= STRIKES) {
                fail("answering very slowly");
                return false;
            }
        } else {
            slowCalls = 0;
            if (down) {
                down = false;   // back again
                problem = "";
            }
        }
        return true;
    }

    public boolean isDown() {
        return down;
    }

    public String problem() {
        return problem;
    }

    private void fail(String how) {
        down = true;
        slowCalls = 0;
        downSinceNs = System.nanoTime();
        problem = what + " " + how + ", so it's off. Check the Expansion Hub cable and power. Driving is not affected; it retries when you use it.";
    }
}
