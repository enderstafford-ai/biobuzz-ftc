package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.Arrays;

/*
 * BioBuzz 38196 - the match TeleOp. Plain mecanum: the wheels do exactly what the sticks say,
 * no gyro or odometry corrections (those were taken out 10/30: they could fight a sideways slide).
 *
 * The screen while driving: problems first (red/orange), the flywheel big, then the match clock (counts down from
 * 2:00; orange at 0:30, red at 0:10) and battery, then intake / rollers / brake / wheels small. Controls list is on INIT.
 * Buzzes: light tick = flywheel speed step, heavy thud = speed can't go further, 2 quick = flywheel stays on,
 * 1 quick = back to normal, 3 long = 30 s left, 1 very long = battery low,
 * 1 long light = field mode on, 2 light ticks = field mode off.
 *
 * DRIVING                                  INIT SCREEN (before ▶), saved for every match after
 *   Left stick   drive + slide               B            intake in/out backward? swap
 *   Right stick  spin                        D-pad ←/→    intake slower/faster
 *   D-pad        slow nudge                  D-pad ↑/↓    how far the brake goes down
 *   LT / LB      intake in / out             X            flywheel spinning backward? flip
 *   RT (hold)    flywheel; screen says READY. Tap twice = stays on, tap once = normal again
 *   Y / A        flywheel faster / slower
 *   R3 (click)   field mode on/off: pushing up = away from you, whichever way the robot faces
 *                (point the robot away from you when you turn it on). Off = normal driving.
 *   BACK (hold)  flywheel slowly backward: frees a ball stuck under it (SHARE on PlayStation-style)
 *   RB (hold)    brake down, wheels stop
 *   X / B (hold) rollers in / out
 */
@TeleOp(name = "Mecanum Drive")   // no group = always first in the Driver Hub list
public class MecanumDrive extends LinearOpMode {

    private static final double DEADZONE = 0.06;      // ignore tiny stick wobble
    private static final double SNAP = 0.2;           // nearly-straight stick pushes count as perfectly straight
    private static final double NUDGE_POWER = 0.25;   // D-pad nudge speed
    private static final double RAMP_PER_SEC = 8.0;   // 1/8 s from stop to full (stopping is instant)
    private static final double LOW_BATTERY = 11.8;
    private static final double TELEOP_SECONDS = 120;   // the clock counts down from ▶, like the field timer
    // 30 seconds left: three long buzzes, both sides (nothing else feels like this)
    private static final Gamepad.RumbleEffect THIRTY_SECONDS_LEFT = new Gamepad.RumbleEffect.Builder()
            .addStep(1, 1, 300).addStep(0, 0, 200)
            .addStep(1, 1, 300).addStep(0, 0, 200)
            .addStep(1, 1, 300).build();
    // Field mode off: two light ticks (on = one long light buzz)
    private static final Gamepad.RumbleEffect FIELD_OFF = new Gamepad.RumbleEffect.Builder()
            .addStep(0, 1, 80).addStep(0, 0, 120).addStep(0, 1, 80).build();

    @Override
    public void runOpMode() {
        DriveWheels wheels = new DriveWheels(hardwareMap);
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO); // faster hardware reads
        }
        Intake intake = new Intake(hardwareMap);
        Brake brake = new Brake(hardwareMap);
        Flywheel flywheel = new Flywheel(hardwareMap);
        Rollers rollers = new Rollers(hardwareMap);
        HeadingSensor gyro = new HeadingSensor(hardwareMap);   // only read while field mode is on
        SelfCheck check = new SelfCheck();                     // which way the gyro counts (Wheel Test step 3)

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        String changed = "";
        while (opModeInInit()) {
            Pad.use(gamepad1);
            if (intake.checkInitButtons(gamepad1)) changed = "Intake setting saved.";
            if (brake.checkAdjustButtons(gamepad1)) changed = "Brake shown at its new down position. It goes back up at ▶.";
            if (flywheel.checkFlipButton(gamepad1)) changed = "Flywheel direction flipped.";
            telemetry.addLine("<big><b>BioBuzz Mecanum Drive</b></big>   Press ▶ to drive.");
            if (!changed.isEmpty()) telemetry.addLine("<b>" + changed + "</b>");
            telemetry.addLine(intake.buttonsHelp());
            telemetry.addLine(brake.help());
            telemetry.addLine(flywheel.help());
            telemetry.addLine(rollers.help());
            telemetry.addLine(Pad.text("Field mode (click the right stick down): " + (fieldModeProblem(gyro, check).isEmpty()
                    ? "ready. Point the robot AWAY from you, then click." : fieldModeProblem(gyro, check))));
            telemetry.addLine(controlsHelp());
            telemetry.update();
            sleep(20);
        }
        brake.raiseAfterAdjusting();

        double[] applied = new double[4];
        double[] amps = new double[4];
        int[] noCurrent = new int[4];   // readings in a row where a wheel was told to go but drew no current
        double volts = battery.getVoltage();
        boolean warnedBattery = false, warned30 = false;
        boolean fieldMode = false, r3Was = false;
        double fieldZero = 0;
        String fieldNote = "";
        int loops = 0;
        ElapsedTime loopTimer = new ElapsedTime();
        ElapsedTime matchTimer = new ElapsedTime();

        while (opModeIsActive()) {
            double dt = Math.min(loopTimer.seconds(), 0.1);
            loopTimer.reset();
            Pad.use(gamepad1);

            double forward = deadzone(-gamepad1.left_stick_y);  // stick up is negative, so flip it
            double strafe  = deadzone(gamepad1.left_stick_x);
            // Thumbs are never perfectly straight: a push that's almost straight up/sideways becomes exactly that
            if (Math.abs(strafe) < SNAP * Math.abs(forward)) strafe = 0;
            if (Math.abs(forward) < SNAP * Math.abs(strafe)) forward = 0;
            // Field mode: click the right stick down. The way the robot faces at that click becomes "away from me",
            // and from then on pushing up drives that way, whichever way the robot is turned. Click again = normal.
            boolean r3 = gamepad1.right_stick_button;
            if (r3 && !r3Was) {
                String why = fieldModeProblem(gyro, check);
                if (fieldMode) {
                    fieldMode = false;
                    gamepad1.runRumbleEffect(FIELD_OFF);
                    fieldNote = "";
                } else if (!why.isEmpty()) {
                    gamepad1.rumble(1.0, 0, 400);   // heavy thud: can't
                    fieldNote = why;
                } else {
                    gyro.update();
                    fieldZero = gyro.heading();
                    fieldMode = true;
                    gamepad1.rumble(0, 1.0, 600);   // one long light buzz: field mode on
                    fieldNote = "";
                }
            }
            r3Was = r3;
            if (fieldMode) {
                gyro.update();
                if (!gyro.available()) {   // gyro dropped out mid-match: straight back to normal driving
                    fieldMode = false;
                    gamepad1.rumble(1.0, 0, 400);
                    fieldNote = "Field mode turned off: " + gyro.problem();
                } else {
                    double turned = Math.toRadians(check.gyroSign * HeadingSensor.wrap(gyro.heading() - fieldZero));   // counterclockwise +
                    double f = forward * Math.cos(turned) - strafe * Math.sin(turned);
                    double s = strafe * Math.cos(turned) + forward * Math.sin(turned);
                    forward = f;
                    strafe = s;
                }
            }
            strafe *= 1.1;                                       // mecanum slides a bit weaker; small boost
            double turn  = deadzone(gamepad1.right_stick_x);
            double power = 1.0;

            boolean nudging = forward == 0 && strafe == 0 && turn == 0
                    && (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right);
            if (nudging) {
                forward = (gamepad1.dpad_up ? 1 : 0) - (gamepad1.dpad_down ? 1 : 0);
                strafe  = (gamepad1.dpad_right ? 1 : 0) - (gamepad1.dpad_left ? 1 : 0);
                power   = NUDGE_POWER;
            }

            double[] target = DriveWheels.mix(forward, strafe, turn);
            for (int i = 0; i < 4; i++) {
                double want = target[i] * power;
                // Ramp up gently (less wheel slip), but slow down / stop right away
                if (want * applied[i] < 0) applied[i] = 0;
                if (Math.abs(want) > Math.abs(applied[i])) {
                    double step = RAMP_PER_SEC * dt;
                    want = Math.max(applied[i] - step, Math.min(applied[i] + step, want));
                }
                applied[i] = want;
            }
            if (brake.update(gamepad1)) Arrays.fill(applied, 0);   // brake down: wheels don't fight it
            wheels.setPowers(applied);
            intake.update(gamepad1);
            flywheel.update(gamepad1);
            rollers.update(gamepad1);

            // Battery and motor current change slowly; read them a few times a second
            if (loops++ % 10 == 0) {
                volts = battery.getVoltage();
                for (int i = 0; i < 4; i++) {
                    amps[i] = wheels.motors[i].getCurrent(CurrentUnit.AMPS);
                    if (Math.abs(applied[i]) > 0.3 && amps[i] < 0.05) noCurrent[i]++;
                    else if (amps[i] > 0.2) noCurrent[i] = 0;
                }
                if (volts < LOW_BATTERY && volts > 1 && !warnedBattery) {
                    gamepad1.rumble(1.0, 0, 1000);   // one long heavy buzz, once
                    warnedBattery = true;
                }
            }

            // Match screen: problems on top, then the shooter big, the clock + battery big, the rest small
            double left = Math.max(0, TELEOP_SECONDS - matchTimer.seconds());
            if (left <= 30 && !warned30) {
                gamepad1.runRumbleEffect(THIRTY_SECONDS_LEFT);
                warned30 = true;
            }
            for (int i = 0; i < 4; i++) {
                if (noCurrent[i] >= 5) telemetry.addLine(String.format(
                        "<font color='#ff5252'><b>%s wheel motor gets no power:</b> its cable is probably out of Control Hub motor port %d.</font>",
                        DriveWheels.LABELS[i], DriveWheels.PORTS[i]));
            }
            for (String t : new String[] {flywheel.trouble(), intake.trouble(), rollers.trouble(), brake.trouble()}) {
                if (!t.isEmpty()) telemetry.addLine(t);
            }
            if (!fieldNote.isEmpty()) telemetry.addLine("<font color='#ffb74d'>" + fieldNote + "</font>");
            if (fieldMode) telemetry.addLine(Pad.text("<big><font color='#4fc3f7'><b>FIELD MODE</b></font></big>  <small>(click right stick = normal)</small>"));
            String big = flywheel.matchLine();
            if (!big.isEmpty()) telemetry.addLine(big);
            String clockColor = left <= 10 ? "#ff5252" : left <= 30 ? "#ffb74d" : "#ffffff";
            String clockWords = left <= 0 ? "  time's up" : left <= 10 ? "  PARK NOW" : left <= 30 ? "  head to the loading zone" : " left";
            telemetry.addLine(String.format("<big><font color='%s'><b>%s</b>%s</font>    <font color='%s'>%.1f V%s</font></big>",
                    clockColor, clock(left), clockWords,
                    volts < LOW_BATTERY ? "#ff5252" : "#69f0ae", volts, volts < LOW_BATTERY ? " LOW, swap after this match" : ""));
            telemetry.addLine("<small>" + intake.compact() + " · " + rollers.compact() + " · " + brake.compact()
                    + (nudging ? " · <font color='#4fc3f7'>nudge</font>" : "") + "</small>");
            telemetry.addLine(String.format("<small><tt><font color='#9e9e9e'>FL %s  FR %s  BL %s  BR %s</font></tt></small>",
                    wheel(applied[0]), wheel(applied[1]), wheel(applied[2]), wheel(applied[3])));
            telemetry.update();
        }
        wheels.stop();
        intake.stop();
        brake.raise();
        flywheel.stop();
        rollers.stop();
    }

    /** Why field mode can't turn on, or "" if it can. */
    private static String fieldModeProblem(HeadingSensor gyro, SelfCheck check) {
        if (!gyro.available()) return gyro.problem().isEmpty() ? "no gyro (needs the Pinpoint or the hub's imu in the configuration)" : gyro.problem();
        if (check.gyroSign == 0) return "the gyro hasn't been checked yet: run Wheel Test step 3 once";
        return "";
    }

    private static double deadzone(double v) {
        return Math.abs(v) < DEADZONE ? 0 : v;
    }

    private static String controlsHelp() {
        return Pad.text("<small><font color='#9e9e9e'>Left stick: drive + slide · Right stick: spin · D-pad: nudge · "
             + "{LT}/{LB}: intake in/out · {X}/{B}: rollers in/out · {RT}: flywheel (2 taps = stays on) · {Y}/{A}: flywheel speed · {BACK}: flywheel backward · {RB}: brake · right stick click: field mode</font></small>");
    }

    private static String wheel(double p) {
        String arrow = p > 0.02 ? "▲" : p < -0.02 ? "▼" : "·";
        return String.format("%s%3d%%", arrow, Math.round(Math.abs(p) * 100));
    }

    private static String clock(double seconds) {
        int s = (int) seconds;
        return String.format("%d:%02d", s / 60, s % 60);
    }
}
