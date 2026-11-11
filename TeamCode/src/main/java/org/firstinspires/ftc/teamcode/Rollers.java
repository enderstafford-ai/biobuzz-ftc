package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ReadWriteFile;

import java.io.File;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/*
 * The two roller wheels (11/10): goBILDA Super Speed servos set to CONTINUOUS mode, separate from the intake for now.
 *   "roller1" = Control Hub servo port 0,   "roller2" = Expansion Hub servo port 1.
 * Both type "Continuous Rotation Servo" in the robot configuration.
 *
 * Hold X: both pull in.   Hold B: both push out (getting POLLEN back out of a FLOWER).   Neither (or both): stopped.
 *
 * They always run together. Which way each one turns is set in the Intake Test op mode (one at a time) and saved
 * on the Control Hub; to start with, roller2 turns the opposite way from roller1. A missing roller is harmless.
 */
public class Rollers {

    static final String[] NAMES = {"roller1", "roller2"};
    private static final File SAVE_FILE = new File("/sdcard/FIRST/biobuzz_rollers.txt");

    private final List<CRServo> servos = new ArrayList<>();
    public final List<String> names = new ArrayList<>();
    private final Set<String> flipped = new HashSet<>();
    private String problem = "";
    private double power;
    private String lastTest = "";
    private final Guard guard = new Guard("Rollers");

    public Rollers(HardwareMap hardwareMap) {
        flipped.add("roller2");   // starting guess: opposite ways
        load();
        for (String name : NAMES) {
            CRServo s;
            try {
                s = hardwareMap.get(CRServo.class, name);
            } catch (RuntimeException e) {
                continue;   // not in the configuration
            }
            // roller2 is on the Expansion Hub: if that hub isn't answering, commands hang, so test it once
            long started = System.nanoTime();
            try {
                s.setPower(0);
            } catch (RuntimeException e) {
                problem = "Roller \"" + name + "\" not answering: is its hub plugged in and powered?";
                continue;
            }
            if (System.nanoTime() - started > 150_000_000L) {
                problem = "Roller \"" + name + "\" answered very slowly: is its hub plugged in and powered?";
                continue;
            }
            servos.add(s);
            names.add(name);
        }
    }

    public boolean available() {
        return !servos.isEmpty();
    }

    public int count() {
        return servos.size();
    }

    /** Call every loop while driving. */
    public void update(Gamepad gamepad) {
        boolean in = gamepad.x, out = gamepad.b;
        double p = in == out ? 0 : in ? 1 : -1;
        if (guard.isDown()) power = Double.NaN;
        if (p == power) return;   // only talk to the hubs when something changes
        if (!guard.ready(p != 0)) return;
        if (guard.run(() -> { for (int i = 0; i < servos.size(); i++) servos.get(i).setPower(direction(i) * p); })) power = p;
    }

    private double direction(int i) {
        return flipped.contains(names.get(i)) ? -1 : 1;
    }

    public boolean isFlipped(int i) {
        return flipped.contains(names.get(i));
    }

    /** Intake Test: reverse just this one roller (saved). */
    public void flip(int i) {
        String name = names.get(i);
        if (!flipped.remove(name)) flipped.add(name);
        save();
    }

    /** Intake Test: spin roller `only` (or both, -1) the IN way at power p; 0 stops. */
    public void test(int only, double p) {
        String want = only + "/" + p;
        if (want.equals(lastTest) && !guard.isDown()) return;
        if (!guard.ready(p != 0)) return;
        if (guard.run(() -> {
            for (int i = 0; i < servos.size(); i++) servos.get(i).setPower(p != 0 && (only < 0 || only == i) ? direction(i) * p : 0);
        })) lastTest = want;
        power = Double.NaN;
    }

    public void stop() {
        if (!guard.isDown()) guard.run(() -> { for (CRServo s : servos) s.setPower(0); });
        power = 0;
    }

    public String status() {
        if (!available()) return "<font color='#ffb74d'>Rollers: " + (problem.isEmpty()
                ? "none in the robot configuration (roller1 = Control Hub servo 0, roller2 = Expansion Hub servo 1)" : problem) + "</font>";
        if (guard.isDown()) return "<font color='#ff5252'>" + guard.problem() + "</font>";
        String doing = power > 0 ? "<font color='#69f0ae'><b>IN</b></font>"
                     : power < 0 ? "<font color='#ff8a65'><b>OUT</b></font>" : "off";
        return Pad.text("Rollers: " + doing + (servos.size() < NAMES.length ? " (only " + names.get(0) + ")" : "")
                + "   <small>({X} in · {B} out)</small>");
    }

    public String help() {
        if (!available()) return status();
        return Pad.text("Rollers (" + String.join(", ", names) + "): hold {X} = in, {B} = out."
                + " Wrong way? Fix each one in the Intake Test op mode."
                + (problem.isEmpty() ? "" : " " + problem));
    }

    private void load() {
        if (!SAVE_FILE.exists()) return;
        try {
            for (String line : ReadWriteFile.readFile(SAVE_FILE).split("\n")) {
                line = line.trim();
                if (line.startsWith("flipped=")) {
                    flipped.clear();
                    for (String n : line.substring(8).split(",")) if (!n.trim().isEmpty()) flipped.add(n.trim());
                }
            }
        } catch (RuntimeException ignored) {
            // A broken file just means the starting settings
        }
    }

    private void save() {
        ReadWriteFile.writeFile(SAVE_FILE, "flipped=" + String.join(",", flipped) + "\n");
    }
}
