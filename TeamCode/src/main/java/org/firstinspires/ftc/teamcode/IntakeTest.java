package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * BioBuzz 38196 - Intake Test: make every intake motor turn the IN way, one motor at a time, so two motors on
 * the same axle never fight. Settings are saved and Mecanum Drive / Simple Drive use them.
 *
 *   D-pad up/down  pick a motor
 *   Hold A         spin ONLY that motor, slowly (the others coast)
 *   B              flip that motor's direction (saved)
 *   Hold Y         all intake motors together, slowly
 *
 * Do it again after changing or re-mounting a motor.
 */
@TeleOp(name = "Intake Test", group = "BioBuzz")
public class IntakeTest extends LinearOpMode {

    private static final double TEST_POWER = 0.3;
    private static final String GOOD = "#69f0ae", DIM = "#9e9e9e";
    private final boolean[] was = new boolean[3];

    @Override
    public void runOpMode() {
        Intake intake = new Intake(hardwareMap);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry.addLine("<big><b>Intake Test</b></big>");
        telemetry.addLine("Checks that each intake motor pulls IN, one at a time. Nothing on the robot drives. Press ▶.");
        if (!intake.available()) telemetry.addLine(intake.status());
        telemetry.update();
        waitForStart();

        int selected = 0;
        while (opModeIsActive()) {
            int n = intake.count();
            if (n > 0) {
                if (pressed(0, gamepad1.dpad_down)) selected = (selected + 1) % n;
                if (pressed(1, gamepad1.dpad_up)) selected = (selected + n - 1) % n;
                if (pressed(2, gamepad1.b)) { intake.flip(selected); gamepad1.rumble(150); }
                if (gamepad1.y) intake.test(-1, TEST_POWER);
                else if (gamepad1.a) intake.test(selected, TEST_POWER);
                else intake.test(-1, 0);
            }

            telemetry.addLine("<big><b>Intake Test</b></big>");
            if (n == 0) {
                telemetry.addLine(intake.status());
            } else {
                for (int i = 0; i < n; i++) {
                    String spinning = (gamepad1.y || (gamepad1.a && i == selected))
                            ? "  <font color='" + GOOD + "'><b>SPINNING</b></font>" : "";
                    telemetry.addLine(String.format("%s Motor %d \"%s\"%s%s  <small><font color='%s'>%s</font></small>",
                            i == selected ? "▶" : "  ", i + 1, intake.names.get(i),
                            intake.isFlipped(i) ? " [flipped]" : "", spinning, DIM, intake.motorReading(i)));
                }
                telemetry.addLine("");
                telemetry.addLine("Hold <b>A</b>: only the ▶ motor spins. It must pull balls IN. If it pushes out, press <b>B</b>.");
                telemetry.addLine("Do it for every motor (D-pad ↑/↓ picks), then hold <b>Y</b>: all together should pull in "
                        + "smoothly. Grinding or buzzing with Y = one is still backward.");
                telemetry.addLine("<small><font color='" + DIM + "'>Everything pushes out instead? Press B on the "
                        + "Mecanum Drive INIT screen (swaps in/out for all of them).</font></small>");
                telemetry.addLine(intake.status());
            }
            telemetry.update();
        }
        intake.stop();
    }

    /** True once per button press. */
    private boolean pressed(int slot, boolean down) {
        boolean edge = down && !was[slot];
        was[slot] = down;
        return edge;
    }
}
