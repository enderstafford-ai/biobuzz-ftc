package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ReadWriteFile;

import java.io.File;

/*
 * The brake under the robot: one servo ("brake", Expansion Hub servo port 0) that turns an axle a little
 * to push the brake pads down onto the floor, so the robot can't be shoved around.
 *
 * Hold RB: brake down (the drive wheels stop while it's down, so they don't fight it).  Let go: brake up.
 *
 * How far it goes down is adjustable without code: on the INIT screen (before ▶), D-pad up/down moves the
 * "down" position (the servo moves there so you can see it, and it's saved on the Control Hub).
 * Starting positions are last year's (team 19439, same robot): up 0.6, down 0.8.
 */
public class Brake {

    private static final double UP = 0.6;
    private static final double STEP = 0.02;
    private static final File SAVE_FILE = new File("/sdcard/FIRST/biobuzz_brake.txt");

    private Servo servo;
    private String problem = "";
    private double down = 0.8;
    private boolean isDown;
    private boolean upWas, downWas;
    private boolean showing;
    private final Guard guard = new Guard("Brake");

    public Brake(HardwareMap hardwareMap) {
        load();
        try {
            servo = hardwareMap.get(Servo.class, "brake");
            // On the Expansion Hub: if that hub isn't answering, commands hang, so test it once and skip it if so
            long started = System.nanoTime();
            servo.setDirection(Servo.Direction.FORWARD);
            servo.setPosition(UP);
            if (System.nanoTime() - started > 150_000_000L) {
                servo = null;
                problem = "Brake servo answered very slowly: is the Expansion Hub plugged in and powered?";
            }
        } catch (RuntimeException e) {
            servo = null;
            problem = "No brake servo: it should be \"brake\" on Expansion Hub servo port 0";
        }
    }

    public boolean available() {
        return servo != null;
    }

    /** INIT screen: D-pad up/down adjusts how far down the brake goes (and shows it). Returns true if changed. */
    public boolean checkAdjustButtons(Gamepad gamepad) {
        if (servo == null) return false;
        boolean more = gamepad.dpad_down && !downWas, less = gamepad.dpad_up && !upWas;
        downWas = gamepad.dpad_down;
        upWas = gamepad.dpad_up;
        if (!more && !less) return false;
        down = Math.max(0, Math.min(1, down + (more ? STEP : -STEP)));
        save();
        guard.run(() -> servo.setPosition(down));     // show where "down" is now
        showing = true;
        return true;
    }

    /** Before ▶: put the brake back up after showing an adjustment. */
    public void raiseAfterAdjusting() {
        if (servo != null && showing) guard.run(() -> servo.setPosition(UP));
        showing = false;
    }

    /** Call every loop while driving. Returns true while the brake is down (stop the drive wheels). */
    public boolean update(Gamepad gamepad) {
        boolean want = gamepad.right_bumper;
        // Only talk to the hub on a change. Holding RB still stops the wheels even if the servo isn't answering.
        if (servo != null && want != isDown && guard.ready(true)) {
            if (guard.run(() -> servo.setPosition(want ? down : UP))) isDown = want;
        }
        return want && servo != null;
    }

    public void raise() {
        if (servo != null && !guard.isDown()) guard.run(() -> servo.setPosition(UP));
        isDown = false;
    }

    public String status() {
        if (servo == null) return "<font color='#ffb74d'>Brake: " + problem + "</font>";
        if (guard.isDown()) return "<font color='#ff5252'>" + guard.problem() + "</font>";
        return isDown ? "<font color='#ff5252'><b>BRAKE DOWN</b></font> (wheels stopped)" : "Brake: up  <small>(hold RB)</small>";
    }

    public String help() {
        if (servo == null) return problem;
        return String.format("Brake: hold RB. Down position %.2f (up %.2f): D-pad down = further, D-pad up = less.", down, UP);
    }

    private void load() {
        if (!SAVE_FILE.exists()) return;
        try {
            String text = ReadWriteFile.readFile(SAVE_FILE).trim();
            if (text.startsWith("down=")) down = Double.parseDouble(text.substring(5));
        } catch (RuntimeException ignored) {
            // A broken file just means the starting position
        }
    }

    private void save() {
        ReadWriteFile.writeFile(SAVE_FILE, String.format("down=%.2f\n", down));
    }
}
