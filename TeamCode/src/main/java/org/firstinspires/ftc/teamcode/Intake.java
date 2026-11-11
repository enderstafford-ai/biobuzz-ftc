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
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/*
 * The intake wheels: a goBILDA 5203 1620 RPM motor ("intake", Expansion Hub motor port 0) through right-angle gears
 * and chains to the wheels, plus a helper motor on the other end of the same axle ("intake2", motor port 2): a 6000 RPM
 * for now, a second 1620 later. Every intake motor runs by power, so any goBILDA motor works with no code change,
 * with or without an encoder (an encoder only adds a speed readout in the Intake Test). A motor whose cable is
 * out just doesn't turn; nothing else changes.
 *
 * Hold LT: intake pulls in.   Hold LB: intake pushes out.   Neither (or both): stopped.
 *
 * Set up on the INIT screen (before ▶), saved on the Control Hub for every match after:
 *   B: swap in/out if they come out backward.   D-pad left/right: intake slower/faster (10% steps).
 * During the match LT and LB are the only intake buttons.
 *
 * Two motors on one axle MUST turn the same way, or they fight (and the weaker one cooks). Each motor's own
 * direction is set in the Intake Test op mode (one motor at a time) and saved here; B above flips them all together.
 *
 * Robot configuration: any motor that isn't one of the four drive wheels (or "flywheel") counts as the intake,
 * whatever it was named, so it works with "intake" or any other name.
 */
public class Intake {

    private static final double MOTOR_RPM = 1620;
    private static final int STEP = 10;           // percent per D-pad press on the INIT screen
    private static final double TRIGGER = 0.3;   // how far LT has to be pulled to count
    private static final File SAVE_FILE = new File("/sdcard/FIRST/biobuzz_intake.txt");

    private static final double ENCODER_FULL_SPEED = 2800;   // counts/s at top speed for EVERY goBILDA motor (28 per motor turn)

    private final List<DcMotorSimple> motors = new ArrayList<>();
    public final List<String> names = new ArrayList<>();
    private final Set<String> flipped = new HashSet<>();      // motors that run the other way from the rest
    private boolean[] sawEncoder;                             // per motor: has its encoder ever counted?
    private String problem = "";
    private boolean swapped;
    private boolean bWasDown, leftWasDown, rightWasDown;
    private int percent = 100;
    private double power;
    private double amps;
    private int statusCalls;
    private String lastTest = "";                             // what Intake Test last asked for, so it is only sent once
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
            if (System.nanoTime() - started > Guard.INIT_TOO_SLOW_NS) {
                problem = "Intake motor \"" + name + "\" answered very slowly: is the Expansion Hub plugged in and powered?";
                continue;
            }
            // Keep them in name order ("intake", then "intake2") so "motor 1" and "motor 2" never change
            int at = 0;
            while (at < names.size() && names.get(at).compareTo(name) < 0) at++;
            motors.add(at, m);
            names.add(at, name);
        }
        sawEncoder = new boolean[motors.size()];
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
        if (guard.run(() -> { for (int i = 0; i < motors.size(); i++) motors.get(i).setPower(direction(i) * p); })) power = p;
    }

    private double direction(int i) {
        return flipped.contains(names.get(i)) ? -1 : 1;
    }

    public int count() {
        return motors.size();
    }

    public boolean isFlipped(int i) {
        return flipped.contains(names.get(i));
    }

    /** Intake Test: reverse just this one motor (saved). */
    public void flip(int i) {
        String name = names.get(i);
        if (!flipped.remove(name)) flipped.add(name);
        save();
    }

    /**
     * Intake Test: spin motor `only` (or all of them, -1) the IN way at power p; 0 stops. The motors left out
     * coast so they don't hold the axle back. Only talks to the hub when something changes.
     */
    public void test(int only, double p) {
        String want = only + "/" + p;
        if (want.equals(lastTest) && !guard.isDown()) return;
        if (!guard.ready(p != 0)) return;
        double in = swapped ? -p : p;
        if (guard.run(() -> {
            for (int i = 0; i < motors.size(); i++) {
                boolean on = p != 0 && (only < 0 || only == i);
                DcMotorSimple m = motors.get(i);
                if (m instanceof DcMotor) ((DcMotor) m).setZeroPowerBehavior(on || p == 0
                        ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
                m.setPower(on ? direction(i) * in : 0);
            }
        })) lastTest = want;
        power = Double.NaN;   // the match code resends its own power afterward
    }

    /** Intake Test: amps and (if it has an encoder) speed of one motor. */
    public String motorReading(int i) {
        DcMotorSimple m = motors.get(i);
        if (!(m instanceof DcMotorEx) || guard.isDown()) return "";
        double[] r = new double[2];
        if (!guard.run(() -> { r[0] = ((DcMotorEx) m).getCurrent(CurrentUnit.AMPS); r[1] = ((DcMotorEx) m).getVelocity(); })) return "";
        if (Math.abs(r[1]) > 20) sawEncoder[i] = true;
        return String.format("%.1f A", r[0]) + (sawEncoder[i]
                ? String.format(", encoder: %.0f%% of top speed", Math.abs(r[1]) / ENCODER_FULL_SPEED * 100)
                : ", no encoder (fine)");
    }

    public void stop() {
        if (!guard.isDown()) guard.run(() -> {
            for (DcMotorSimple m : motors) {
                if (m instanceof DcMotor) ((DcMotor) m).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                m.setPower(0);
            }
        });
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
        return Pad.text(String.format("Intake: %s  <b>%d%%</b> %s %.0f motor RPM  %.1fA   <small>({LT} in · {LB} out)</small>",
                doing, percent, bar(), MOTOR_RPM * percent / 100, amps));
    }

    public String buttonsHelp() {
        if (!available()) return problem.isEmpty() ? "No intake motor found. Add the intake motor to the robot configuration." : problem;
        return Pad.text("Intake (" + String.join(", ", names) + "): hold {LT} = in, {LB} = out. Speed " + percent + "% (D-pad ←/→ now)."
             + (swapped ? " [swapped]" : "") + " Backward? Press {B} now."
             + (motors.size() > 1 ? " Motors fighting each other? Fix it in the Intake Test op mode." : ""));
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
                if (line.startsWith("flipped=")) {
                    for (String n : line.substring(8).split(",")) if (!n.trim().isEmpty()) flipped.add(n.trim());
                }
            }
        } catch (RuntimeException ignored) {
            // A broken file just means the starting settings
        }
    }

    private void save() {
        ReadWriteFile.writeFile(SAVE_FILE, "swapped=" + swapped + "\nspeed=" + percent
                + "\nflipped=" + String.join(",", flipped) + "\n");
    }
}
