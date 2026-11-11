package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.File;
import java.util.Arrays;

/*
 * BioBuzz 38196 - autonomous: LEAVE (3 points) + PARK in our LOADING ZONE (5 points).
 * Works for red and blue alike; the only thing that changes is which side the Loading Zone is on,
 * so there are two op modes: "Auto: Zone on my LEFT" and "Auto: Zone on my RIGHT".
 *
 * Setup: back of the robot against the wall in front of our drivers, beside the Loading Zone
 * (not in it), 4 POLLEN touching the robot. "Left"/"right" = as seen from behind the robot,
 * the same way the drivers see it.
 *
 * What it does:
 *   1. waits the start delay (so a partner can go first),
 *   2. drives forward 6 in. off the wall (LEAVE),
 *   3. slides sideways into the Loading Zone (PARK), then stops and waits for TELEOP.
 *
 * On the INIT screen, with gamepad 1 (settings are saved on the robot):
 *   D-pad up/down     slide distance (how far the Loading Zone is), 3 in. steps
 *   D-pad left/right  start delay, 1 s steps
 *
 * Distances come from the odometry pods when they're working; if the pods don't count, it falls back
 * to driving for a time instead. If something blocks the robot (another robot, a wall), the pods see it
 * stop moving: it lets off the power, waits a second, tries once more, then gives up and just stops,
 * so it never keeps shoving.
 */
public abstract class ParkAuto extends LinearOpMode {

    protected abstract int zoneSide();   // -1 = Loading Zone to the left, +1 = to the right

    private static final File SAVE_FILE = new File("/sdcard/FIRST/biobuzz_auto.txt");

    private static final double LEAVE_INCHES = 6;
    private static final double FORWARD_POWER = 0.35;
    private static final double SLIDE_POWER = 0.45;
    private static final double RAMP_PER_SEC = 2.0;        // gentle start so the wheels don't slip
    private static final double HOLD_KP = 0.03;            // keeps the robot facing straight while it moves
    private static final double HOLD_MAX = 0.25;
    // Time fallback when the pods don't count: rough speeds at the powers above (inches per second)
    private static final double FORWARD_IN_PER_SEC = 25;
    private static final double SLIDE_IN_PER_SEC = 22;
    private static final double STALL_SECONDS = 1.0;       // no movement this long = blocked

    private double slideInches = 12;          // starting beside the zone, 12 in. puts the robot well over the tape
    private double delaySeconds = 0;

    private DriveWheels wheels;
    private HeadingSensor gyro;
    private SelfCheck check;
    private double startHeading;
    private final double[] applied = new double[4];
    private String status = "";

    @Override
    public void runOpMode() {
        wheels = new DriveWheels(hardwareMap);
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        gyro = new HeadingSensor(hardwareMap);
        check = new SelfCheck();
        load();

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        boolean upWas = false, downWas = false, leftWas = false, rightWas = false;
        while (opModeInInit()) {
            boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down, left = gamepad1.dpad_left, right = gamepad1.dpad_right;
            boolean changed = true;
            if (up && !upWas) slideInches = Math.min(96, slideInches + 3);
            else if (down && !downWas) slideInches = Math.max(0, slideInches - 3);
            else if (right && !rightWas) delaySeconds = Math.min(20, delaySeconds + 1);
            else if (left && !leftWas) delaySeconds = Math.max(0, delaySeconds - 1);
            else changed = false;
            if (changed) save();
            upWas = up; downWas = down; leftWas = left; rightWas = right;

            gyro.update();
            telemetry.addLine(String.format("<big><b>Auto: Loading Zone on my %s</b></big>", zoneSide() < 0 ? "LEFT" : "RIGHT"));
            telemetry.addLine("Back against the wall, beside the Loading Zone (not in it), 4 POLLEN touching.");
            telemetry.addLine(String.format("Slide: <b>%.0f in.</b>  <small>(D-pad up/down)</small>", slideInches));
            telemetry.addLine(String.format("Wait first: <b>%.0f s</b>  <small>(D-pad left/right)</small>", delaySeconds));
            telemetry.addLine("Distances: " + (gyro.hasPosition() ? "odometry pods" : "<font color='#ffb74d'>by time (no Pinpoint)</font>")
                    + "   Straight: " + (gyro.available() && check.gyroSign != 0 ? "gyro"
                    : "<font color='#ffb74d'>off (do step 3 of the Wheel Test op mode once so the gyro is checked)</font>"));
            if (!gyro.problem().isEmpty()) telemetry.addLine("<font color='#ffb74d'>" + gyro.problem() + "</font>");
            telemetry.addLine("Keep the robot still. Press ▶ when the match starts.");
            telemetry.update();
            sleep(20);
        }
        if (!opModeIsActive()) return;

        gyro.update();
        startHeading = gyro.heading();
        ElapsedTime wait = new ElapsedTime();
        while (opModeIsActive() && wait.seconds() < delaySeconds) {
            show(String.format("Waiting %.0f s", delaySeconds - wait.seconds()));
            sleep(20);
        }

        move("Leaving the wall", 1, 0, LEAVE_INCHES);
        move("Sliding into the Loading Zone", 0, zoneSide(), slideInches);
        wheels.stop();

        while (opModeIsActive()) {
            show("<font color='#69f0ae'><b>Done: parked.</b></font> " + status);
            sleep(50);
        }
    }

    /** Drive one straight leg: forward (1) or sideways (-1 left, +1 right), for a distance in inches. */
    private void move(String what, int forward, int sideways, double inches) {
        if (inches <= 0 || !opModeIsActive()) return;
        boolean slide = sideways != 0;
        double power = slide ? SLIDE_POWER : FORWARD_POWER;
        double timeLimit = inches / (slide ? SLIDE_IN_PER_SEC : FORWARD_IN_PER_SEC);
        boolean usePods = gyro.hasPosition();
        int startTicks = podTicks(slide);
        int stallTicks = startTicks;
        int tries = 0;
        ElapsedTime legTimer = new ElapsedTime();
        ElapsedTime stallTimer = new ElapsedTime();
        ElapsedTime loopTimer = new ElapsedTime();
        double moved = 0;
        int ticks = startTicks;

        while (opModeIsActive()) {
            double dt = Math.min(loopTimer.seconds(), 0.1);
            loopTimer.reset();
            gyro.update();
            if (gyro.hasPosition()) {   // keep the last good count if the Pinpoint just dropped out
                ticks = podTicks(slide);
                moved = Math.abs(ticks - startTicks) / HeadingSensor.POD_TICKS_PER_INCH;
            }

            // Pods not counting at all after half a second of driving: they're not working, use time instead
            if (usePods && legTimer.seconds() > 0.5 && moved < 0.5 && tries == 0) {
                usePods = false;
                status = "(pods didn't count, went by time)";
            }
            // Pinpoint dropped out partway (loose cable): finish the rest of this leg by time
            if (usePods && !gyro.hasPosition()) {
                usePods = false;
                timeLimit = legTimer.seconds() + Math.max(0, inches - moved) / (slide ? SLIDE_IN_PER_SEC : FORWARD_IN_PER_SEC);
                status = "(Pinpoint dropped out, finished by time)";
            }
            if (usePods ? moved >= inches : legTimer.seconds() >= timeLimit) break;
            if (legTimer.seconds() > timeLimit * 3 + 2) {          // way too long: something's wrong, stop
                status = "(" + what.toLowerCase() + " took too long, stopped)";
                break;
            }

            // Blocked? The pods stopped moving while we're pushing
            if (usePods) {
                if (Math.abs(ticks - stallTicks) / HeadingSensor.POD_TICKS_PER_INCH > 0.3) {
                    stallTicks = ticks;
                    stallTimer.reset();
                } else if (stallTimer.seconds() > STALL_SECONDS) {
                    if (++tries > 1) {
                        status = "(blocked, gave up " + Math.round(inches - moved) + " in. short)";
                        break;
                    }
                    wheels.stop();
                    Arrays.fill(applied, 0);
                    show("Blocked, waiting a second...");
                    sleep(1000);
                    stallTicks = podTicks(slide);
                    stallTimer.reset();
                    loopTimer.reset();
                    continue;
                }
            }

            // Slow down for the last 4 inches so it stops where it should
            double scale = usePods ? Math.max(0.5, Math.min(1, (inches - moved) / 4)) : 1;
            double f = forward * power * scale, s = sideways * power * scale * 1.1;
            double turn = 0;
            if (check.gyroSign != 0 && gyro.available()) {
                double error = check.gyroSign * HeadingSensor.wrap(gyro.heading() - startHeading);
                turn = clip(HOLD_KP * error, HOLD_MAX);
            }
            drive(f, s, turn, dt);
            show(String.format("%s: %.1f / %.0f in.%s", what, usePods ? moved : legTimer.seconds() / timeLimit * inches,
                    inches, usePods ? "" : " (by time)"));
        }
        wheels.stop();
        Arrays.fill(applied, 0);
        sleep(150);   // let it settle before the next leg
    }

    /** Sends one loop's worth of drive power, ramping up gently so the wheels don't slip. */
    private void drive(double forward, double strafe, double turn, double dt) {
        double[] target = DriveWheels.mix(forward, strafe, turn);
        double step = RAMP_PER_SEC * dt;
        for (int i = 0; i < 4; i++) {
            double want = target[i];
            if (Math.abs(want) > Math.abs(applied[i])) want = Math.max(applied[i] - step, Math.min(applied[i] + step, want));
            applied[i] = want;
        }
        wheels.setPowers(applied);
    }

    private int podTicks(boolean sideways) {
        return sideways ? gyro.yPodTicks() : gyro.xPodTicks();
    }

    private void show(String line) {
        telemetry.addLine(String.format("<big><b>Auto: zone on my %s</b></big>", zoneSide() < 0 ? "LEFT" : "RIGHT"));
        telemetry.addLine(line);
        telemetry.update();
    }

    private static double clip(double v, double limit) {
        return Math.max(-limit, Math.min(limit, v));
    }

    private void load() {
        if (!SAVE_FILE.exists()) return;
        try {
            for (String line : ReadWriteFile.readFile(SAVE_FILE).split("\n")) {
                String[] parts = line.trim().split("=");
                if (parts.length != 2) continue;
                if (parts[0].equals("slide")) slideInches = Double.parseDouble(parts[1]);
                if (parts[0].equals("delay")) delaySeconds = Double.parseDouble(parts[1]);
            }
        } catch (RuntimeException ignored) {
            // A broken file just means the starting settings
        }
    }

    private void save() {
        ReadWriteFile.writeFile(SAVE_FILE, "slide=" + slideInches + "\ndelay=" + delaySeconds + "\n");
    }
}
