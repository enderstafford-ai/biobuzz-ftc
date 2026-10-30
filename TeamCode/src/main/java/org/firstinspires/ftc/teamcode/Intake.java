package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * The intake wheels: one goBILDA 5203 motor, 1620 RPM, through right-angle gears and chains to the wheels.
 *
 * Hold LT: intake pulls in.   Hold LB: intake pushes out.   Neither (or both): stopped.
 *
 * Set up on the INIT screen (before ▶), saved on the Control Hub for every match after:
 *   B: swap in/out if they come out backward.   D-pad left/right: intake slower/faster (10% steps).
 * During the match LT and LB are the only intake buttons.
 *
 * Robot configuration: any motor that isn't one of the four drive wheels (or "flywheel") counts as the intake,
 * whatever it was named, so it works with "intake" or any other name.
 */
public class Intake {

    private static final double MOTOR_RPM = 1620;
    private static final int STEP = 10;           // percent per D-pad press on the INIT screen
    private static final double TRIGGER = 0.3;   // how far LT has to be pulled to count
    private static final File SAVE_FILE = new File("/sdcard/FIRST/biobuzz_intake.txt");

    private final List<DcMotorSimple> motors = new ArrayList<>();
    public final List<String> names = new ArrayList<>();
    private String problem = "";
    private boolean swapped;
    private boolean bWasDown, leftWasDown, rightWasDown;
    private int percent = 100;
    private double power;
    private double amps;
    private int statusCalls;
    private final Guard guard = new Guard("Intake");

    public Intake(HardwareMap hardwareMap) {
        List<String> drive = Arrays.asList(DriveWheels.NAMES);
        for (DcMotorSimple m : hardwareMap.getAll(DcMotorSimple.class)) {
            if (m instanceof CRServo) continue;
            String name = hardwareMap.getNamesOf(m).isEmpty() ? "?" : hardwareMap.getNamesOf(m).iterator().next();
            if (drive.contains(name) || name.equals("flywheel")) continue;   // the flywheel has its own controls
            // The intake sits on the Expansion Hub. If that hub isn't plugged in or powered, every command to it
            // hangs until it times out, which would slow the whole robot down, so test it once and skip it if so.
            long started = System.nanoTime();
            try {
                if (m instanceof DcMotor) {
                    ((DcMotor) m).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    ((DcMotor) m).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
                m.setPower(0);
            } catch (RuntimeException e) {
                problem = "Intake motor \"" + name + "\" not answering: is the Expansion Hub plugged in and powered?";
                continue;
            }
            if (System.nanoTime() - started > 150_000_000L) {
                problem = "Intake motor \"" + name + "\" answered very slowly: is the Expansion Hub plugged in and powered?";
                continue;
            }
            motors.add(m);
            names.add(name);
        }
        load();
    }

    public boolean available() {
        return !motors.isEmpty();
    }

    /** Call every loop on the INIT screen: B swaps in/out, D-pad left/right = speed. Saved. True if changed. */
    public boolean checkInitButtons(Gamepad gamepad) {
        boolean swap = gamepad.b && !bWasDown;
        boolean faster = gamepad.dpad_right && !rightWasDown, slower = gamepad.dpad_left && !leftWasDown;
        bWasDown = gamepad.b;
        rightWasDown = gamepad.dpad_right;
        leftWasDown = gamepad.dpad_left;
        if (!swap && !faster && !slower) return false;
        if (swap) swapped = !swapped;
        int before = percent;
        if (faster || slower) percent = Math.max(STEP, Math.min(100, percent + (faster ? STEP : -STEP)));
        gamepad.rumble(swap || percent != before ? 120 : 400);   // long buzz = already at the top/bottom
        save();
        return true;
    }

    /** Call every loop while driving. */
    public void update(Gamepad gamepad) {
        boolean in = gamepad.left_trigger > TRIGGER;
        boolean out = gamepad.left_bumper;
        double speed = percent / 100.0;
        double pull = in == out ? 0 : in ? speed : -speed;
        double p = swapped ? -pull : pull;
        if (guard.isDown()) power = Double.NaN;   // unknown while the hub isn't answering: resend on the next try
        if (p == power) return;   // only talk to the hub when something changes
        if (!guard.ready(p != 0)) return;
        if (guard.run(() -> { for (DcMotorSimple m : motors) m.setPower(p); })) power = p;
    }

    public void stop() {
        if (!guard.isDown()) guard.run(() -> { for (DcMotorSimple m : motors) m.setPower(0); });
        power = 0;
    }

    /** One line for the screen: what it's doing and how hard the motor is working. */
    public String status() {
        if (!available()) {
            return "<font color='#ffb74d'>Intake: " + (problem.isEmpty() ? "no intake motor in the robot configuration" : problem) + "</font>";
        }
        if (guard.isDown()) return "<font color='#ff5252'>" + guard.problem() + "</font>";
        double sign = swapped ? -power : power;
        String doing = sign > 0 ? "<font color='#69f0ae'><b>IN</b></font>"
                     : sign < 0 ? "<font color='#ff8a65'><b>OUT</b></font>" : "off";
        if (statusCalls++ % 10 == 0) {   // current changes slowly; reading it every loop slows the hub down
            guard.run(() -> {
                double total = 0;
                for (DcMotorSimple m : motors) if (m instanceof DcMotorEx) total += ((DcMotorEx) m).getCurrent(CurrentUnit.AMPS);
                amps = total;
            });
        }
        return String.format("Intake: %s  <b>%d%%</b> %s %.0f motor RPM  %.1fA   <small>(LT in · LB out)</small>",
                doing, percent, bar(), MOTOR_RPM * percent / 100, amps);
    }

    public String buttonsHelp() {
        if (!available()) return problem.isEmpty() ? "No intake motor found. Add the intake motor to the robot configuration." : problem;
        return "Intake (" + String.join(", ", names) + "): hold LT = in, LB = out. Speed " + percent + "% (D-pad ←/→ now)."
             + (swapped ? " [swapped]" : "") + " Backward? Press B now.";
    }

    private String bar() {
        StringBuilder b = new StringBuilder();
        for (int i = 10; i <= 100; i += 10) b.append(i <= percent ? '█' : '░');
        return b.toString();
    }

    private void load() {
        if (!SAVE_FILE.exists()) return;
        try {
            for (String line : ReadWriteFile.readFile(SAVE_FILE).split("\n")) {
                line = line.trim();
                if (line.equals("swapped")) swapped = true;                    // older one-word file
                if (line.startsWith("swapped=")) swapped = line.endsWith("true");
                if (line.startsWith("speed=")) percent = Math.max(STEP, Math.min(100, Integer.parseInt(line.substring(6))));
            }
        } catch (RuntimeException ignored) {
            // A broken file just means the starting settings
        }
    }

    private void save() {
        ReadWriteFile.writeFile(SAVE_FILE, "swapped=" + swapped + "\nspeed=" + percent + "\n");
    }
}
