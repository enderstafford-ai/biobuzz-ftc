package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ReadWriteFile;

import java.io.File;

/*
 * The shooter flywheel: goBILDA 96 mm Gecko wheel (50A) straight on a goBILDA 6000 RPM motor (no gearbox,
 * SKU 5000-0002-4008), "flywheel" on Expansion Hub motor port 1, encoder cable in Expansion Hub encoder port 1.
 * Shoots out the back of the robot.
 *
 * Hold RT: flywheel spins up and holds its speed (the encoder keeps it there between shots), off when let go.
 *          The controller buzzes once when it's up to speed: then LT feeds balls in.
 * Tap RT twice quickly: it stays on without holding (two buzzes). One tap of RT: back to normal (hold to spin).
 * Y / A: flywheel faster / slower (100 RPM steps), any time while driving.
 * The speed and direction are saved on the Control Hub.
 *
 * The wheel's side facing the intake must move UP. If it spins the other way, press X on the INIT screen.
 *
 * Velocity tuning (PIDF) and the 28 counts per turn are last year's (team 19439), same motor.
 */
public class Flywheel {

    private static final double TICKS_PER_REV = 28;           // 6000 RPM motor, encoder on the motor itself
    private static final int START_RPM = 3000;                // ball leaves at about 6.8 m/s (needs ~6 to reach the hive)
    private static final int MIN_RPM = 1000, MAX_RPM = 5800, STEP_RPM = 100;
    private static final int READY_TOLERANCE_RPM = 100;
    private static final double FREE_RPM = 6000;              // motor's top speed, for running by power alone
    private static final long DOUBLE_TAP_NS = 400_000_000L;   // second tap within this = stay on
    private static final long ENCODER_CHECK_NS = 1_500_000_000L;   // spinning this long with no encoder counts = its cable is out
    private static final long INIT_TOO_SLOW_NS = 2 * Guard.INIT_TOO_SLOW_NS;   // INIT sends several commands, so allow double
    private static final File SAVE_FILE = new File("/sdcard/FIRST/biobuzz_flywheel.txt");

    private DcMotorEx motor;
    private String problem = "";
    private int targetRpm = START_RPM;
    private boolean reversed;
    private boolean xWas, yWas, aWas, rtWas;
    private boolean locked;          // double-tapped: stays on until RT is tapped again
    private long lastTapNs;
    private boolean spinning, buzzed;
    private boolean mayBeOn;   // a command failed while spinning: keep trying to stop it after RT is let go
    private boolean noEncoder;       // encoder cable missing: run by plain power instead of chasing a speed it can't see
    private long spinStartedNs;
    private final Guard guard = new Guard("Flywheel");
    private double rpm;

    public Flywheel(HardwareMap hardwareMap) {
        load();
        try {
            motor = hardwareMap.get(DcMotorEx.class, "flywheel");
            long started = System.nanoTime();
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);   // coast down, don't slam the wheel
            motor.setVelocityPIDFCoefficients(40, 0.01, 4, 14.5);
            motor.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            if (System.nanoTime() - started > INIT_TOO_SLOW_NS) {
                motor = null;
                problem = "Flywheel motor answered very slowly: is the Expansion Hub plugged in and powered?";
            }
        } catch (RuntimeException e) {
            motor = null;
            problem = "No flywheel motor: it should be \"flywheel\" on Expansion Hub motor port 1";
        }
    }

    public boolean available() {
        return motor != null;
    }

    /** INIT screen: X flips which way the flywheel spins (saved). Returns true when pressed. */
    public boolean checkFlipButton(Gamepad gamepad) {
        boolean pressed = gamepad.x && !xWas;
        xWas = gamepad.x;
        if (!pressed || motor == null) return false;
        reversed = !reversed;
        guard.run(() -> motor.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD));
        save();
        gamepad.rumble(200);
        return true;
    }

    /** Call every loop while driving. */
    public void update(Gamepad gamepad) {
        boolean held = gamepad.right_trigger > 0.3;
        if (held && !rtWas) {   // a new tap
            long now = System.nanoTime();
            if (locked) {
                locked = false;   // one tap: back to normal (still spins while this tap is held)
                gamepad.rumble(80);
                lastTapNs = 0;
            } else if (lastTapNs != 0 && now - lastTapNs < DOUBLE_TAP_NS) {
                locked = true;
                gamepad.rumbleBlips(2);
                lastTapNs = 0;
            } else {
                lastTapNs = now;
            }
        }
        rtWas = held;
        boolean want = held || locked;
        boolean changed = false;
        boolean faster = gamepad.y && !yWas, slower = gamepad.a && !aWas;
        if (faster || slower) {
            int before = targetRpm;
            targetRpm = Math.max(MIN_RPM, Math.min(MAX_RPM, targetRpm + (faster ? STEP_RPM : -STEP_RPM)));
            gamepad.rumble(targetRpm == before ? 400 : 80);
            save();
            changed = true;
            buzzed = false;
        }
        yWas = gamepad.y;
        aWas = gamepad.a;
        if (motor == null || !guard.ready(want || mayBeOn)) return;
        boolean speedChanged = changed;
        if (!guard.run(() -> drive(want, speedChanged))) {
            spinning = false;   // hub not answering: start over next time
            mayBeOn = true;
        } else {
            mayBeOn = false;
        }
        if (spinning && !buzzed && ready()) {
            gamepad.rumble(250);   // up to speed: feed with LT
            buzzed = true;
        }
    }

    private void drive(boolean want, boolean speedChanged) {
        if (want && !spinning) {
            command();
            buzzed = false;
            spinStartedNs = System.nanoTime();
        } else if (want && speedChanged) {
            command();
        } else if (!want && (spinning || mayBeOn)) {
            motor.setPower(0);
        }
        spinning = want;

        rpm = Math.abs(motor.getVelocity()) / TICKS_PER_REV * 60;
        // Spinning 1.5 s and the encoder still reads ~0: its cable is out, and chasing the speed would mean full power
        if (spinning && !noEncoder && rpm < 200 && System.nanoTime() - spinStartedNs > ENCODER_CHECK_NS) {
            noEncoder = true;
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            command();
        }
    }

    private void command() {
        if (noEncoder) motor.setPower(targetRpm / FREE_RPM);
        else motor.setVelocity(targetRpm / 60.0 * TICKS_PER_REV);
    }

    public boolean ready() {
        if (noEncoder) return spinning && System.nanoTime() - spinStartedNs > ENCODER_CHECK_NS;
        return spinning && Math.abs(rpm - targetRpm) <= READY_TOLERANCE_RPM;
    }

    public void stop() {
        locked = false;
        if (motor != null && !guard.isDown()) guard.run(() -> motor.setPower(0));
        spinning = false;
    }

    public String status() {
        if (motor == null) return "<font color='#ffb74d'>Flywheel: " + problem + "</font>";
        if (guard.isDown()) return "<font color='#ff5252'>" + guard.problem() + "</font>";
        if (noEncoder) return String.format("Flywheel: <font color='#ffb74d'>no encoder signal (plug its small cable into Expansion Hub encoder port 1)</font>, running at about %d RPM by power  %s",
                targetRpm, spinning ? "<b>ON</b>" : Pad.text("<small>(hold {RT})</small>"));
        String state = !spinning ? "off  <small>(hold {RT} · tap twice = stays on)</small>"
                : locked && ready() ? "<font color='#69f0ae'><b>READY</b></font> <b>· STAYS ON</b>, feed with {LT} <small>(tap {RT} to stop)</small>"
                : locked ? "<font color='#ffb74d'>spinning up</font> <b>· STAYS ON</b> <small>(tap {RT} to stop)</small>"
                : ready() ? "<font color='#69f0ae'><b>READY</b></font>, feed with {LT}"
                : "<font color='#ffb74d'>spinning up</font>";
        return Pad.text(String.format("Flywheel: %s   %.0f / <b>%d RPM</b>  <small>({Y}/{A}: speed)</small>", state, rpm, targetRpm));
    }

    public String help() {
        if (motor == null) return problem;
        return Pad.text("Flywheel: hold {RT} to spin (" + targetRpm + " RPM), tap {RT} twice = stays on, tap once = back to normal, {Y}/{A} = speed. Spinning the wrong way? Press {X} now.");
    }

    private void load() {
        if (!SAVE_FILE.exists()) return;
        try {
            for (String line : ReadWriteFile.readFile(SAVE_FILE).split("\n")) {
                line = line.trim();
                if (line.startsWith("rpm=")) targetRpm = Math.max(MIN_RPM, Math.min(MAX_RPM, Integer.parseInt(line.substring(4))));
                if (line.startsWith("reversed=")) reversed = line.endsWith("true");
            }
        } catch (RuntimeException ignored) {
            // A broken file just means the starting settings
        }
    }

    private void save() {
        ReadWriteFile.writeFile(SAVE_FILE, "rpm=" + targetRpm + "\nreversed=" + reversed + "\n");
    }
}
