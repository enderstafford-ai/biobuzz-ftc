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
 * 1 quick = back to normal, 3 long = 30 s left, 1 very long = battery low.
 *
 * DRIVING                                  INIT SCREEN (before ▶), saved for every match after
 *   Left stick   drive + slide               B            intake in/out backward? swap
 *   Right stick  spin                        D-pad ←/→    intake slower/faster
 *   D-pad        slow nudge                  D-pad ↑/↓    how far the brake goes down
 *   LT / LB      intake in / out             X            flywheel spinning backward? flip
 *   RT (hold)    flywheel; screen says READY. Tap twice = stays on, tap once = normal again
 *   Y / A        flywheel faster / slower
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

    private static double deadzone(double v) {
        return Math.abs(v) < DEADZONE ? 0 : v;
    }

    private static String controlsHelp() {
        return Pad.text("<small><font color='#9e9e9e'>Left stick: drive + slide · Right stick: spin · D-pad: nudge · "
             + "{LT}/{LB}: intake in/out · {X}/{B}: rollers in/out · {RT}: flywheel (2 taps = stays on) · {Y}/{A}: flywheel speed · {BACK}: flywheel backward · {RB}: brake</font></small>");
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
