package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/*
 * BioBuzz 38196 - mecanum drive (Logitech F310 controller).
 *
 * Left stick:  drive forward/back and slide left/right.
 * Right stick (side to side): spin in place.
 * D-pad: slow nudge while held, for lining up (only when the sticks are let go).
 * X button (blue): cycle speed  Normal -> Fast -> Slow -> Normal.
 * BACK button: heading hold on/off. While on, the gyro (Pinpoint, or the Control Hub's) keeps the robot pointing
 *   the same way whenever you aren't using the spin stick, so it drives straight instead of drifting.
 *
 * Nothing here moves the robot on its own: it only moves while you hold a stick or the D-pad.
 * Wheel directions come from DriveWheels (fix a backward wheel with the Wheel Test op mode).
 */
@TeleOp(name = "Mecanum Drive", group = "BioBuzz")
public class MecanumDrive extends LinearOpMode {

    // Speed modes: name and how much of full power the sticks give
    private static final String[] MODE_NAMES = {"Normal", "Fast", "Slow"};
    private static final double[] MODE_POWER = {0.65, 1.0, 0.35};

    private static final double DEADZONE = 0.06;      // ignore tiny stick wobble so "straight up" is really straight
    private static final double NUDGE_POWER = 0.25;   // D-pad nudge speed
    private static final double RAMP_PER_SEC = 4.0;   // speeding up takes 1/4 s from stop to full (stopping is instant)
    private static final double LOW_BATTERY = 11.8;
    private static final double SNAP = 0.2;           // nearly-straight stick pushes count as perfectly straight
    private static final double HOLD_KP = 0.03;       // heading hold strength: turn power per degree off course
    private static final double HOLD_MAX = 0.4;       // most turning heading hold may add
    private static final double HOLD_SETTLE = 0.3;    // seconds after you let go of spin before it locks the heading

    @Override
    public void runOpMode() {
        DriveWheels wheels = new DriveWheels(hardwareMap);
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO); // faster hardware reads
        }

        HeadingSensor gyro = new HeadingSensor(hardwareMap);

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry.addLine("<big><b>BioBuzz Mecanum Drive</b></big>");
        telemetry.addLine(gyro.available() ? "Gyro: " + gyro.sourceName() + ". Keep the robot still until you press ▶."
                : "<font color='#ffb74d'>No \"pinpoint\" or \"imu\" in the configuration, so heading hold is off.</font>");
        telemetry.addLine("Ready. Press ▶ to drive.");
        telemetry.addLine(controlsHelp());
        telemetry.update();
        waitForStart();

        int mode = 0;               // start in Normal
        boolean xWasDown = false;
        double[] applied = new double[4];
        double[] amps = new double[4];
        double volts = battery.getVoltage();
        boolean warnedBattery = false;
        int loops = 0;
        boolean holdOn = gyro.available();
        boolean backWasDown = false;
        double heading = 0;
        Double holdTarget = null;
        double holdCorrection = 0;
        ElapsedTime sinceSpin = new ElapsedTime();
        ElapsedTime loopTimer = new ElapsedTime();
        ElapsedTime matchTimer = new ElapsedTime();

        while (opModeIsActive()) {
            double dt = Math.min(loopTimer.seconds(), 0.1);
            loopTimer.reset();

            // X (blue): change speed once per press (not every loop while held)
            boolean xDown = gamepad1.x;
            if (xDown && !xWasDown) {
                mode = (mode + 1) % MODE_NAMES.length;
                gamepad1.rumble(150);
            }
            xWasDown = xDown;

            // BACK: heading hold on/off
            boolean backDown = gamepad1.back;
            if (backDown && !backWasDown && gyro.available()) {
                holdOn = !holdOn;
                holdTarget = null;
                gamepad1.rumble(holdOn ? 100 : 300);
            }
            backWasDown = backDown;

            double forward = deadzone(-gamepad1.left_stick_y);  // stick up is negative, so flip it
            double strafe  = deadzone(gamepad1.left_stick_x);
            // Thumbs are never perfectly straight: a push that's almost straight up/sideways becomes exactly that
            if (Math.abs(strafe) < SNAP * Math.abs(forward)) strafe = 0;
            if (Math.abs(forward) < SNAP * Math.abs(strafe)) forward = 0;
            strafe *= 1.1;                                       // mecanum slides a bit weaker; small boost
            double turn    = deadzone(gamepad1.right_stick_x);
            double power   = MODE_POWER[mode];

            boolean sticksIdle = forward == 0 && strafe == 0 && turn == 0;
            boolean nudging = sticksIdle && (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right);
            if (nudging) {
                forward = (gamepad1.dpad_up ? 1 : 0) - (gamepad1.dpad_down ? 1 : 0);
                strafe  = (gamepad1.dpad_right ? 1 : 0) - (gamepad1.dpad_left ? 1 : 0);
                power   = NUDGE_POWER;
            }

            // Heading hold: while you're driving but not spinning, steer back to the way the robot was pointing
            gyro.update();
            heading = gyro.heading();
            boolean driving = forward != 0 || strafe != 0;
            if (turn != 0 || !holdOn) {
                sinceSpin.reset();
                holdTarget = null;
            } else if (holdTarget == null && sinceSpin.seconds() > HOLD_SETTLE) {
                holdTarget = heading;
            }
            holdCorrection = 0;
            if (holdTarget != null && driving) {
                double error = wrap(heading - holdTarget);   // positive = robot has rotated left
                holdCorrection = Math.max(-HOLD_MAX, Math.min(HOLD_MAX, HOLD_KP * error / power));
                turn = holdCorrection;
            }

            double[] target = {
                forward + strafe + turn,   // front left
                forward - strafe - turn,   // front right
                forward - strafe + turn,   // back left
                forward + strafe - turn,   // back right
            };

            // Keep every wheel within -1..1 without changing the direction of travel
            double biggest = 1.0;
            for (double t : target) biggest = Math.max(biggest, Math.abs(t));
            for (int i = 0; i < 4; i++) {
                double want = target[i] * power / biggest;
                // Ramp up gently (less wheel slip and tipping), but slow down / stop right away
                if (want * applied[i] < 0) applied[i] = 0;   // reversing: stop first, then ramp the other way
                if (Math.abs(want) > Math.abs(applied[i])) {
                    double step = RAMP_PER_SEC * dt;
                    want = Math.max(applied[i] - step, Math.min(applied[i] + step, want));
                }
                applied[i] = want;
            }
            wheels.setPowers(applied[0], applied[1], applied[2], applied[3]);

            // Battery and motor current change slowly; read them a few times a second
            if (loops++ % 10 == 0) {
                volts = battery.getVoltage();
                for (int i = 0; i < 4; i++) amps[i] = wheels.motors[i].getCurrent(CurrentUnit.AMPS);
                if (volts < LOW_BATTERY && volts > 1 && !warnedBattery) {
                    gamepad1.rumble(600);
                    warnedBattery = true;
                }
            }

            telemetry.addLine(String.format("<big><b>Speed: <font color='%s'>%s</font></b></big>  %s %d%%",
                    modeColor(mode), MODE_NAMES[mode], bar(MODE_POWER[mode]), Math.round(MODE_POWER[mode] * 100)));
            if (nudging) telemetry.addLine("<font color='#4fc3f7'>D-pad nudge</font>");
            telemetry.addLine(String.format("Battery: <font color='%s'><b>%.2f V</b></font>%s    Time: %s",
                    volts < LOW_BATTERY ? "#ff5252" : "#69f0ae", volts,
                    volts < LOW_BATTERY ? "  LOW, swap soon" : "", clock(matchTimer.seconds())));
            telemetry.addLine(!gyro.available() ? "Heading hold: <font color='#ffb74d'>no gyro</font>"
                    : String.format("Heading hold: %s   Facing %.0f°%s",
                        holdOn ? "<font color='#69f0ae'><b>ON</b></font>" : "<font color='#9e9e9e'>off</font>",
                        heading, holdCorrection != 0 ? String.format("   correcting %+.0f°", wrap(heading - holdTarget)) : ""));
            if (gyro.hasPosition()) {
                String st = gyro.status();
                telemetry.addLine(String.format("Pinpoint: x %.1f in  y %.1f in  %s", gyro.xInches(), gyro.yInches(),
                        st.equals("READY") ? "" : "<font color='#ffb74d'>" + st + "</font>"));
            }
            telemetry.addLine("<b>Wheels</b> (power, amps)");
            telemetry.addLine(String.format("<tt>FL %s %4.1fA    FR %s %4.1fA</tt>", wheel(applied[0]), amps[0], wheel(applied[1]), amps[1]));
            telemetry.addLine(String.format("<tt>BL %s %4.1fA    BR %s %4.1fA</tt>", wheel(applied[2]), amps[2], wheel(applied[3]), amps[3]));
            telemetry.addLine(String.format("<small>Sticks: fwd %.2f  slide %.2f  spin %.2f   Loop %d ms</small>",
                    forward, strafe, turn, Math.round(dt * 1000)));
            telemetry.addLine(controlsHelp());
            telemetry.update();
        }
        wheels.stop();
    }

    private static double deadzone(double v) {
        return Math.abs(v) < DEADZONE ? 0 : v;
    }

    /** Angle difference folded into -180..180 degrees. */
    private static double wrap(double degrees) {
        while (degrees > 180) degrees -= 360;
        while (degrees < -180) degrees += 360;
        return degrees;
    }

    private static String controlsHelp() {
        return "<small><font color='#9e9e9e'>Left stick: drive + slide · Right stick: spin · "
             + "D-pad: nudge · X: speed · BACK: heading hold</font></small>";
    }

    private static String modeColor(int mode) {
        return mode == 1 ? "#ff8a65" : mode == 2 ? "#4fc3f7" : "#ffffff";
    }

    private static String bar(double fraction) {
        int filled = (int) Math.round(fraction * 10);
        StringBuilder s = new StringBuilder();
        for (int i = 0; i < 10; i++) s.append(i < filled ? '█' : '░');
        return s.toString();
    }

    private static String wheel(double p) {
        String arrow = p > 0.02 ? "▲" : p < -0.02 ? "▼" : "·";
        return String.format("%s%4d%%", arrow, Math.round(Math.abs(p) * 100));
    }

    private static String clock(double seconds) {
        int s = (int) seconds;
        return String.format("%d:%02d", s / 60, s % 60);
    }
}
