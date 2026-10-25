package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.Arrays;

/*
 * BioBuzz 38196 - Wheel Test: a step-by-step check of the whole drivetrain.
 * Bumpers move between steps (RB = next, LB = back).
 *
 * 1. Which wheel is which (robot on a box, wheels in the air)
 *      D-pad up/down: pick a wheel.  Hold A: spin only that wheel.  B: flip its direction (saved).
 *      X: "this one's right" (the highlighted wheel spun, rolling forward).  Y: all four forward.
 * 2. Roller pattern: a picture of what the wheels should look like from above.  X = looks right, B = doesn't.
 * 3. Sensors, by hand (robot on the floor, wheels free): push it forward, push it right, turn it clockwise.
 *      Finds which way the odometry pods and gyro count, and saves it for Mecanum Drive.  Y: start over.
 * 4. Floor test: hold A = drive forward, hold B = slide right, hold X = spin clockwise, all slow.
 *      The robot ONLY moves while you hold the button. The pods measure what it really did.
 * 5. Results.
 */
@TeleOp(name = "Wheel Test", group = "BioBuzz")
public class WheelTest extends LinearOpMode {

    private static final double BOX_POWER = 0.3;
    private static final double FLOOR_POWER = 0.25;
    private static final double TICKS_PER_INCH = 505.3;   // goBILDA 4-bar pod
    private static final String[] STEPS = {"Which wheel is which", "Roller pattern", "Sensors (push by hand)",
                                           "Floor test", "Results"};
    private static final String GOOD = "#69f0ae", BAD = "#ff8a65", WARN = "#ffd54f", DIM = "#9e9e9e";

    private DriveWheels wheels;
    private HeadingSensor gyro;
    private SelfCheck check;
    private final boolean[] was = new boolean[8];

    // Step 1
    private int selected = 0;
    private final boolean[] wheelOk = new boolean[4];
    private final double[] boxAmps = new double[4];
    // Step 2
    private String pattern = "";
    // Step 3
    private int handStage = 0;          // 0 push forward, 1 push right, 2 turn clockwise, 3 done
    private int startX, startY, measuredX, measuredY;
    private boolean podsSwapped;
    private double turned, lastHeading;
    private String sensorsResult = "";
    // Step 4
    private String floorTest = "";       // "forward", "slide" or "spin" while its button is held
    private double movedFwd, movedRight, turnedCw, testTime;
    private int lastX, lastY;
    private final double[] ampSum = new double[4];
    private int ampSamples;
    private String forwardResult = "", slideResult = "", spinResult = "", ampResult = "";

    @Override
    public void runOpMode() {
        wheels = new DriveWheels(hardwareMap);
        gyro = new HeadingSensor(hardwareMap);
        check = new SelfCheck();
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        telemetry.addLine("<big><b>Wheel Test</b></big>");
        telemetry.addLine("Step 1 needs the robot on a box, wheels in the air. Keep it still, then press ▶.");
        telemetry.update();
        waitForStart();

        int step = 0;
        ElapsedTime loopTimer = new ElapsedTime();
        while (opModeIsActive()) {
            double dt = Math.min(loopTimer.seconds(), 0.1);
            loopTimer.reset();
            gyro.update();

            if (pressed(0, gamepad1.right_bumper) && step < STEPS.length - 1) enterStep(++step);
            if (pressed(1, gamepad1.left_bumper) && step > 0) enterStep(--step);

            telemetry.addLine(String.format("<big><b>Wheel Test</b></big>  Step %d of %d: <b>%s</b>",
                    step + 1, STEPS.length, STEPS[step]));
            telemetry.addLine("<small><font color='" + DIM + "'>RB = next step · LB = back</font></small>");

            if (step == 0) wheelStep();
            else if (step == 1) patternStep();
            else if (step == 2) handStep();
            else if (step == 3) floorStep(dt);
            else resultsStep();

            telemetry.update();
        }
        wheels.stop();
        wheels.setFreeWheel(false);
    }

    private void enterStep(int step) {
        wheels.stop();
        wheels.setFreeWheel(step == 2);   // free wheels so the robot can be pushed by hand
        if (step == 2) startHandStage(0);
        floorTest = "";
    }

    /** True once per button press. */
    private boolean pressed(int slot, boolean down) {
        boolean edge = down && !was[slot];
        was[slot] = down;
        return edge;
    }

    /** Angle difference folded into -180..180 degrees. */
    private static double wrap(double degrees) {
        while (degrees > 180) degrees -= 360;
        while (degrees < -180) degrees += 360;
        return degrees;
    }

    // ---------------------------------------------------------------- 1. Which wheel is which

    private void wheelStep() {
        if (pressed(2, gamepad1.dpad_down)) selected = (selected + 1) % 4;
        if (pressed(3, gamepad1.dpad_up)) selected = (selected + 3) % 4;
        if (pressed(4, gamepad1.b)) {
            wheels.flip(selected);
            wheelOk[selected] = false;
            gamepad1.rumble(200);
        }
        if (pressed(5, gamepad1.x)) {
            wheelOk[selected] = true;
            gamepad1.rumble(100);
            selected = (selected + 1) % 4;
        }

        double[] p = new double[4];
        if (gamepad1.y) Arrays.fill(p, BOX_POWER);
        else if (gamepad1.a) p[selected] = BOX_POWER;
        wheels.setPowers(p[0], p[1], p[2], p[3]);
        for (int i = 0; i < 4; i++) {
            if (p[i] != 0) boxAmps[i] = wheels.motors[i].getCurrent(CurrentUnit.AMPS);
        }

        telemetry.addLine("Robot on a box, wheels in the air.  <small>(front of robot is up)</small>");
        telemetry.addLine("<tt>" + wheelCell(0, p) + "  " + wheelCell(1, p) + "</tt>");
        telemetry.addLine("<tt>" + wheelCell(2, p) + "  " + wheelCell(3, p) + "</tt>");
        telemetry.addLine(String.format("Selected: <b>%s</b> (port %d)", DriveWheels.LABELS[selected], DriveWheels.PORTS[selected]));
        telemetry.addLine("<b>Hold A.</b> Check BOTH:");
        telemetry.addLine("  • the <b>highlighted</b> wheel is the one spinning (not another one)");
        telemetry.addLine("  • it rolls the way the robot drives <b>forward</b>");
        telemetry.addLine("Both right? Press <b>X</b> (it moves to the next wheel).");
        telemetry.addLine("Rolls backward? Press <b>B</b> to flip it.");
        telemetry.addLine("A <i>different</i> wheel spins? Its cable is in the wrong port. Ports: "
                + "front left 0, back left 1, front right 2, back right 3. "
                + "<small>(Front right and back left swapped is the usual one: swap the cables in ports 1 and 2.)</small>");
        String amps = ampsWarning(boxAmps, 1.0);
        if (!amps.isEmpty()) telemetry.addLine("<font color='" + WARN + "'>" + amps + "</font>");
        telemetry.addLine("<small>D-pad: pick wheel · Y: all four forward</small>");
    }

    private String wheelCell(int wheel, double[] p) {
        String mark = wheelOk[wheel] ? " ✓" : (p[wheel] > 0 ? " ▲" : "  ");
        String text = "[" + String.format("%-11s", DriveWheels.LABELS[wheel]) + mark + "]";
        if (wheel == selected) return "<font color='" + WARN + "'><b>" + text + "</b></font>";
        return wheelOk[wheel] ? "<font color='" + GOOD + "'>" + text + "</font>" : text;
    }

    // ---------------------------------------------------------------- 2. Roller pattern

    private void patternStep() {
        if (pressed(5, gamepad1.x)) pattern = "looks right";
        if (pressed(4, gamepad1.b)) pattern = "WRONG";

        telemetry.addLine("Look straight DOWN at the wheels from above, front of the robot away from you.");
        telemetry.addLine("The rollers on TOP of each wheel should slant like this:");
        telemetry.addLine("<tt>     FRONT</tt>");
        telemetry.addLine("<tt>   \\       /</tt>");
        telemetry.addLine("<tt>   /       \\</tt>");
        telemetry.addLine("<tt>     BACK</tt>");
        telemetry.addLine("Easy rule: the 2 wheels on the SAME side are <b>mirror images</b>; "
                + "wheels on OPPOSITE corners look <b>identical</b>.");
        telemetry.addLine("If both left wheels look the same, two wheels are on the wrong corners. "
                + "It drives forward fine but slides badly, and step 4 will show it.");
        telemetry.addLine("<b>X</b> = looks right · <b>B</b> = doesn't.  "
                + (pattern.isEmpty() ? "" : "You said: <b>" + pattern + "</b>"));
    }

    // ---------------------------------------------------------------- 3. Sensors, pushed by hand

    private void startHandStage(int stage) {
        handStage = gyro.hasPosition() || stage >= 2 ? stage : 2;   // no Pinpoint: only the gyro to check
        startX = gyro.xPodTicks();
        startY = gyro.yPodTicks();
        turned = 0;
        lastHeading = gyro.heading();
    }

    private void handStep() {
        if (pressed(6, gamepad1.y)) {
            sensorsResult = "";
            podsSwapped = false;
            measuredX = measuredY = 0;
            startHandStage(0);
        }
        double dx = (gyro.xPodTicks() - startX) / TICKS_PER_INCH;
        double dy = (gyro.yPodTicks() - startY) / TICKS_PER_INCH;
        turned += wrap(gyro.heading() - lastHeading);
        lastHeading = gyro.heading();

        telemetry.addLine("Robot on the floor. The wheels roll freely now; push it with your hands.");
        if (!gyro.problem().isEmpty()) telemetry.addLine("<font color='" + WARN + "'>" + gyro.problem() + "</font>");

        if (handStage == 0) {
            telemetry.addLine("<big>① Push the robot <b>FORWARD</b> about 1 foot.</big>");
            if (Math.max(Math.abs(dx), Math.abs(dy)) >= 6) {
                podsSwapped |= Math.abs(dy) > Math.abs(dx);
                measuredX = dx > 0 ? 1 : -1;
                gamepad1.rumble(150);
                startHandStage(1);
            }
        } else if (handStage == 1) {
            telemetry.addLine("<big>② Push the robot to the <b>RIGHT</b> (sideways) about 1 foot.</big>");
            if (Math.max(Math.abs(dx), Math.abs(dy)) >= 6) {
                podsSwapped |= Math.abs(dx) > Math.abs(dy);
                measuredY = dy > 0 ? 1 : -1;
                gamepad1.rumble(150);
                startHandStage(2);
            }
        } else if (handStage == 2) {
            telemetry.addLine("<big>③ Turn the robot <b>CLOCKWISE</b> (to the right) about a quarter turn.</big>");
            if (!gyro.available()) {
                sensorsResult = "no gyro or Pinpoint answering";
                handStage = 3;
            } else if (Math.abs(turned) >= 45) {
                int measuredGyro = turned < 0 ? 1 : -1;   // clockwise should make the heading go DOWN
                if (podsSwapped) {
                    check.setMeasured(measuredGyro, 0, 0);
                    sensorsResult = "PODS SWAPPED: the X and Y cables on the Pinpoint are in each other's ports. "
                            + "Swap them, then press Y and do this step again.";
                } else {
                    check.setMeasured(measuredGyro, measuredX, measuredY);
                    sensorsResult = "saved. Gyro " + (measuredGyro > 0 ? "normal" : "counts backward (fine, now known)")
                            + (gyro.hasPosition() ? ", forward pod " + (measuredX > 0 ? "normal" : "reversed (fine, now known)")
                               + ", sideways pod " + (measuredY > 0 ? "normal" : "reversed (fine, now known)") : ", no pods");
                }
                gamepad1.rumble(400);
                handStage = 3;
            }
        }
        if (handStage < 3) {
            telemetry.addLine(String.format("<small>Moved: forward pod %.1f in, sideways pod %.1f in, turned %.0f°</small>",
                    dx, dy, Math.abs(turned)));
        } else {
            boolean bad = !sensorsResult.startsWith("saved");
            telemetry.addLine("<font color='" + (bad ? BAD : GOOD) + "'><b>Sensors " + sensorsResult + "</b></font>");
            telemetry.addLine("Press RB for the floor test.  <small>Y = do this step again</small>");
        }
    }

    // ---------------------------------------------------------------- 4. Floor test

    private void floorStep(double dt) {
        String want = gamepad1.a ? "forward" : gamepad1.b ? "slide" : gamepad1.x ? "spin" : "";
        int x = gyro.xPodTicks(), y = gyro.yPodTicks();

        if (!want.equals(floorTest)) {
            if (!floorTest.isEmpty()) judge(floorTest);   // button let go: give the verdict
            floorTest = want;
            movedFwd = movedRight = turnedCw = testTime = 0;
            Arrays.fill(ampSum, 0);
            ampSamples = 0;
        }

        double f = 0, s = 0, t = 0;
        if (floorTest.equals("forward")) f = FLOOR_POWER;
        if (floorTest.equals("slide")) s = FLOOR_POWER;
        if (floorTest.equals("spin")) t = FLOOR_POWER;
        wheels.setPowers(f + s + t, f - s - t, f - s + t, f + s - t);   // same mixing as Mecanum Drive

        if (!floorTest.isEmpty()) {
            testTime += dt;
            if (check.xPodSign != 0) movedFwd += check.xPodSign * (x - lastX) / TICKS_PER_INCH;
            if (check.yPodSign != 0) movedRight += check.yPodSign * (y - lastY) / TICKS_PER_INCH;
            if (check.gyroSign != 0) turnedCw += -check.gyroSign * wrap(gyro.heading() - lastHeading);
            for (int i = 0; i < 4; i++) ampSum[i] += wheels.motors[i].getCurrent(CurrentUnit.AMPS);
            ampSamples++;
        }
        lastX = x;
        lastY = y;
        lastHeading = gyro.heading();

        telemetry.addLine("Robot on the floor with room around it. It moves ONLY while you hold:");
        telemetry.addLine("  <b>A</b> = drive forward · <b>B</b> = slide right · <b>X</b> = spin clockwise");
        telemetry.addLine("Hold each one for about 2 feet (1 second for the spin), then let go.");
        if (check.gyroSign == 0 || (gyro.hasPosition() && (check.xPodSign == 0 || check.yPodSign == 0))) {
            telemetry.addLine("<font color='" + WARN + "'>Do step 3 first so the pods and gyro can measure this.</font>");
        }
        if (!gyro.hasPosition()) {
            telemetry.addLine("<font color='" + WARN + "'>No Pinpoint: watch the robot yourself. "
                    + "Slide right should go straight right without turning or creeping forward.</font>");
        }
        if (!floorTest.isEmpty()) {
            telemetry.addLine(String.format("<b>Now:</b> forward %.1f in · right %.1f in · turned %.0f° clockwise",
                    movedFwd, movedRight, turnedCw));
        }
        telemetry.addLine("Drive forward: " + show(forwardResult));
        telemetry.addLine("Slide right: " + show(slideResult));
        telemetry.addLine("Spin: " + show(spinResult));
        if (!ampResult.isEmpty()) telemetry.addLine("<font color='" + WARN + "'>" + ampResult + "</font>");
    }

    private void judge(String test) {
        double[] avg = new double[4];
        for (int i = 0; i < 4; i++) avg[i] = ampSamples == 0 ? 0 : ampSum[i] / ampSamples;
        String amps = ampsWarning(avg, 0.5);
        if (!amps.isEmpty()) ampResult = amps;

        boolean pods = gyro.hasPosition() && check.xPodSign != 0 && check.yPodSign != 0;
        if (test.equals("forward")) {
            if (!pods) { forwardResult = "?watch it: did it go straight?"; return; }
            if (movedFwd <= -2) { forwardResult = "!it went BACKWARD. Redo step 1 (wheel directions)."; return; }
            if (movedFwd < 6) { forwardResult = "?hold A longer (2 feet)"; return; }
            double drift = Math.abs(movedRight) / movedFwd * 12, turn = Math.abs(turnedCw) / movedFwd * 12;
            forwardResult = drift < 1.5 && turn < 8
                    ? String.format("ok: straight (%.1f in sideways per foot)", drift)
                    : String.format("!drifts %.1f in sideways and turns %.0f° per foot", drift, turn);
        } else if (test.equals("slide")) {
            if (!pods) { slideResult = "?watch it: did it go straight right?"; return; }
            if (movedRight <= -2) {
                slideResult = "!it slid LEFT. The rollers make an O, not an X: the wheels are on the wrong corners "
                        + "(swap the left and right wheels). See step 2.";
                return;
            }
            if (Math.abs(movedRight) < 3 && testTime > 1) {
                slideResult = "!barely slides. Most likely 2 wheels on the wrong corners (step 2) or a slipping belt.";
                return;
            }
            if (movedRight < 6) { slideResult = "?hold B longer (2 feet)"; return; }
            double drift = Math.abs(movedFwd) / movedRight * 12, turn = Math.abs(turnedCw) / movedRight * 12;
            slideResult = drift < 3 && turn < 10
                    ? String.format("ok: straight (%.1f in forward/back per foot)", drift)
                    : String.format("!goes diagonal: %.1f in %s and turns %.0f° per foot. Likely one wheel on the wrong "
                            + "corner (step 2) or a slipping belt (check the motor note).",
                            drift, movedFwd > 0 ? "forward" : "backward", turn);
        } else if (test.equals("spin")) {
            if (check.gyroSign == 0) { spinResult = "?watch it: did it turn in place?"; return; }
            if (testTime < 0.7) { spinResult = "?hold X longer (1 second)"; return; }
            double rate = turnedCw / testTime;
            if (rate < 0) spinResult = "!turned the WRONG way (counterclockwise). Redo step 1.";
            else if (rate < 40) spinResult = String.format("!weak turning (%.0f°/s). Check step 1 (front right and "
                    + "back left swapped?) and step 2.", rate);
            else spinResult = String.format("ok: %.0f° per second", rate);
            if (pods && Math.abs(movedFwd) + Math.abs(movedRight) > 6 && !spinResult.startsWith("!")) {
                spinResult = "!turns but also slides around (" + Math.round(Math.abs(movedFwd) + Math.abs(movedRight))
                        + " in). A wheel is slipping: check step 2 and the belts.";
            }
        }
    }

    /** A wheel drawing far less current than the others isn't pushing (belt slipping/off); far more = rubbing. */
    private static String ampsWarning(double[] amps, double minAmps) {
        double[] sorted = amps.clone();
        Arrays.sort(sorted);
        double middle = (sorted[1] + sorted[2]) / 2;
        if (middle < minAmps * 0.3) return "";
        StringBuilder s = new StringBuilder();
        for (int i = 0; i < 4; i++) {
            if (amps[i] == 0) continue;
            if (amps[i] < middle * 0.4) s.append(DriveWheels.LABELS[i]).append(String.format(
                    " barely working (%.1f A): belt slipping or off, or gearbox loose. ", amps[i]));
            if (amps[i] > middle * 2.2) s.append(DriveWheels.LABELS[i]).append(String.format(
                    " working too hard (%.1f A): something rubbing, or belt too tight. ", amps[i]));
        }
        return s.toString();
    }

    private static String show(String result) {
        if (result.isEmpty()) return "<font color='" + DIM + "'>not tried</font>";
        if (result.startsWith("!")) return "<font color='" + BAD + "'><b>" + result.substring(1) + "</b></font>";
        if (result.startsWith("?")) return "<font color='" + WARN + "'>" + result.substring(1) + "</font>";
        return "<font color='" + GOOD + "'>" + result + "</font>";
    }

    // ---------------------------------------------------------------- 5. Results

    private void resultsStep() {
        int ok = 0;
        for (boolean b : wheelOk) if (b) ok++;
        telemetry.addLine("1. Wheels: " + (ok == 4 ? show("ok: all 4 checked") : show("?" + ok + " of 4 checked")));
        telemetry.addLine("2. Roller pattern: " + (pattern.isEmpty() ? show("")
                : pattern.equals("WRONG") ? show("!you said it looks wrong") : show("ok: looks right")));
        telemetry.addLine("3. Sensors: " + (sensorsResult.isEmpty() ? show("")
                : sensorsResult.startsWith("saved") ? show("ok: " + sensorsResult) : show("!" + sensorsResult)));
        telemetry.addLine("4. Drive forward: " + show(forwardResult));
        telemetry.addLine("    Slide right: " + show(slideResult));
        telemetry.addLine("    Spin: " + show(spinResult));
        if (!ampResult.isEmpty()) telemetry.addLine("    Motors: " + show("!" + ampResult));
        telemetry.addLine("<small>Take a photo of this screen and send it over.</small>");
    }
}
