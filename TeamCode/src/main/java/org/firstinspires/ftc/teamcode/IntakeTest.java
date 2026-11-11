package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * BioBuzz 38196 - Intake Test: make every intake motor and roller turn the IN way, one at a time, so two motors on
 * the same axle never fight. Settings are saved and Mecanum Drive / Simple Drive use them.
 *
 *   D-pad up/down  pick a motor or roller
 *   Hold A         spin ONLY that one, slowly (the others coast)
 *   B              flip that one's direction (saved)
 *   Hold Y         all intake motors together, slowly
 *   Hold X         both rollers together
 *
 * Do it again after changing or re-mounting a motor.
 */
@TeleOp(name = "Intake Test", group = "BioBuzz")
public class IntakeTest extends LinearOpMode {

    private static final double TEST_POWER = 0.3;
    private static final double ROLLER_POWER = 0.5;
    private static final String GOOD = "#69f0ae", DIM = "#9e9e9e";
    private final boolean[] was = new boolean[3];

    @Override
    public void runOpMode() {
        Intake intake = new Intake(hardwareMap);
        Rollers rollers = new Rollers(hardwareMap);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry.addLine("<big><b>Intake Test</b></big>");
        telemetry.addLine("Checks that each intake motor and roller pulls IN, one at a time. Nothing on the robot drives. Press ▶.");
        if (!intake.available()) telemetry.addLine(intake.status());
        if (!rollers.available()) telemetry.addLine(rollers.status());
        telemetry.update();
        waitForStart();

        int selected = 0;
        while (opModeIsActive()) {
            Pad.use(gamepad1);
            int motors = intake.count(), n = motors + rollers.count();   // rollers come after the motors in the list
            if (n > 0) {
                if (pressed(0, gamepad1.dpad_down)) selected = (selected + 1) % n;
                if (pressed(1, gamepad1.dpad_up)) selected = (selected + n - 1) % n;
                boolean roller = selected >= motors;
                if (pressed(2, gamepad1.b)) {
                    if (roller) rollers.flip(selected - motors); else intake.flip(selected);
                    gamepad1.rumble(150);
                }
                if (gamepad1.y) intake.test(-1, TEST_POWER);
                else if (gamepad1.a && !roller) intake.test(selected, TEST_POWER);
                else intake.test(-1, 0);
                if (gamepad1.x) rollers.test(-1, ROLLER_POWER);
                else if (gamepad1.a && roller) rollers.test(selected - motors, ROLLER_POWER);
                else rollers.test(-1, 0);
            }

            telemetry.addLine("<big><b>Intake Test</b></big>");
            if (n == 0) {
                telemetry.addLine(intake.status());
                telemetry.addLine(rollers.status());
            } else {
                for (int i = 0; i < motors; i++) {
                    String spinning = (gamepad1.y || (gamepad1.a && i == selected))
                            ? "  <font color='" + GOOD + "'><b>SPINNING</b></font>" : "";
                    telemetry.addLine(String.format("%s Motor %d \"%s\"%s%s  <small><font color='%s'>%s</font></small>",
                            i == selected ? "▶" : "  ", i + 1, intake.names.get(i),
                            intake.isFlipped(i) ? " [flipped]" : "", spinning, DIM, intake.motorReading(i)));
                }
                for (int r = 0; r < rollers.count(); r++) {
                    int i = motors + r;
                    String spinning = (gamepad1.x || (gamepad1.a && i == selected))
                            ? "  <font color='" + GOOD + "'><b>SPINNING</b></font>" : "";
                    telemetry.addLine(String.format("%s Roller \"%s\"%s%s", i == selected ? "▶" : "  ",
                            rollers.names.get(r), rollers.isFlipped(r) ? " [flipped]" : "", spinning));
                }
                if (!intake.available()) telemetry.addLine(intake.status());
                if (!rollers.available()) telemetry.addLine(rollers.status());
                telemetry.addLine("");
                telemetry.addLine(Pad.text("Hold <b>{A}</b>: only the ▶ one spins. It must pull balls IN. If it pushes out, press <b>{B}</b>."));
                telemetry.addLine(Pad.text("Do it for every motor and roller (D-pad ↑/↓ picks), then hold <b>{Y}</b>: all intake motors "
                        + "should pull in smoothly (grinding or buzzing = one is still backward). Hold <b>{X}</b>: both rollers pull in."));
                telemetry.addLine("<small><font color='" + DIM + "'>Everything pushes out instead? Press " + Pad.text("{B}") + " on the "
                        + "Mecanum Drive INIT screen (swaps in/out for all of them).</font></small>");
                telemetry.addLine(intake.status());
            }
            telemetry.update();
        }
        intake.stop();
        rollers.stop();
    }

    /** True once per button press. */
    private boolean pressed(int slot, boolean down) {
        boolean edge = down && !was[slot];
        was[slot] = down;
        return edge;
    }
}
