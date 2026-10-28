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
 * Y: intake faster.   A: intake slower.   (10% steps, 10% to 100%; the screen shows the speed.)
 *
 * The speed and which way is "in" are saved in a small file on the Control Hub, so the speed you
 * settle on stays for the next match. If LT and LB come out backward, press B on the INIT screen
 * (before ▶) to swap them. No code change or reinstall needed.
 *
 * Robot configuration: any motor that isn't one of the four drive wheels (or "flywheel") counts as the intake,
 * whatever it was named, so it works with "intake" or any other name.
 */
public class Intake {

    private static final double MOTOR_RPM = 1620;
    private static final int STEP = 10;           // percent per Y/A press
    private static final double TRIGGER = 0.3;   // how far LT has to be pulled to count
    private static final File SAVE_FILE = new File("/sdcard/FIRST/biobuzz_intake.txt");

    private final List<DcMotorSimple> motors = new ArrayList<>();
    public final List<String> names = new ArrayList<>();
    private String problem = "";
    private boolean swapped;
    private boolean bWasDown, yWasDown, aWasDown;
    private int percent = 100;
    private double power;
    private double amps;
    private int statusCalls;

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

    /** Call every loop on the INIT screen: B swaps which button pulls in, and saves it. */
    public boolean checkSwapButton(Gamepad gamepad) {
        boolean pressed = gamepad.b && !bWasDown;
        bWasDown = gamepad.b;
        if (pressed) {
            swapped = !swapped;
            save();
            gamepad.rumble(200);
        }
        return pressed;
    }

    /** Call every loop while driving. speedButtons = false while RT is held (Y/A then belong to the flywheel). */
    public void update(Gamepad gamepad, boolean speedButtons) {
        // Y / A: change the speed once per press (not every loop while held)
        boolean faster = speedButtons && gamepad.y && !yWasDown, slower = speedButtons && gamepad.a && !aWasDown;
        yWasDown = gamepad.y;
        aWasDown = gamepad.a;
        if (faster || slower) {
            int before = percent;
            percent = Math.max(STEP, Math.min(100, percent + (faster ? STEP : -STEP)));
            gamepad.rumble(percent == before ? 400 : 80);   // long buzz = already at the top/bottom
            save();
        }

        boolean in = gamepad.left_trigger > TRIGGER;
        boolean out = gamepad.left_bumper;
        double speed = percent / 100.0;
        double p = in == out ? 0 : in ? speed : -speed;
        if (swapped) p = -p;
        if (p == power) return;   // only talk to the hub when something changes
        power = p;
        for (DcMotorSimple m : motors) m.setPower(power);
    }

    public void stop() {
        power = 0;
        for (DcMotorSimple m : motors) m.setPower(0);
    }

    /** One line for the screen: what it's doing and how hard the motor is working. */
    public String status() {
        if (!available()) {
            return "<font color='#ffb74d'>Intake: " + (problem.isEmpty() ? "no intake motor in the robot configuration" : problem) + "</font>";
        }
        double sign = swapped ? -power : power;
        String doing = sign > 0 ? "<font color='#69f0ae'><b>IN</b></font>"
                     : sign < 0 ? "<font color='#ff8a65'><b>OUT</b></font>" : "off";
        if (statusCalls++ % 10 == 0) {   // current changes slowly; reading it every loop slows the hub down
            amps = 0;
            for (DcMotorSimple m : motors) if (m instanceof DcMotorEx) amps += ((DcMotorEx) m).getCurrent(CurrentUnit.AMPS);
        }
        return String.format("Intake: %s  <b>%d%%</b> %s %.0f motor RPM  %.1fA   <small>(LT in · LB out · Y/A speed)</small>",
                doing, percent, bar(), MOTOR_RPM * percent / 100, amps);
    }

    public String buttonsHelp() {
        if (!available()) return problem.isEmpty() ? "No intake motor found. Add the intake motor to the robot configuration." : problem;
        return "Intake (" + String.join(", ", names) + "): hold LT = in, LB = out, Y/A = faster/slower (now "
             + percent + "%)." + (swapped ? " [swapped]" : "") + " Backward? Press B now to swap.";
    }

    /** Plain-text speed for screens without HTML. */
    public String speedText() {
        return String.format("Intake speed %d%% (%.0f motor RPM), Y/A to change", percent, MOTOR_RPM * percent / 100);
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
