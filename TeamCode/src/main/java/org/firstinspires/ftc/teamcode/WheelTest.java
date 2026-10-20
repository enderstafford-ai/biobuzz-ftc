package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * BioBuzz 38196 - Wheel Test. Put the robot on a box, wheels in the air.
 *
 * D-pad up/down: pick a wheel.
 * Hold A: spin ONLY that wheel slowly. It should roll the way the robot drives forward.
 * Press B: that wheel spins backward? Flip it. Saved on the robot right away, and
 *          Mecanum Drive uses it next time you press INIT. No code change needed.
 * Hold Y: all four wheels slowly forward, to double-check.
 */
@TeleOp(name = "Wheel Test", group = "BioBuzz")
public class WheelTest extends LinearOpMode {

    private static final double TEST_POWER = 0.3;

    @Override
    public void runOpMode() {
        DriveWheels wheels = new DriveWheels(hardwareMap);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        int selected = 0;
        boolean upWas = false, downWas = false, bWas = false;
        String message = "";
        int[] startTicks = new int[4];
        boolean[] encoderSeen = new boolean[4];
        for (int i = 0; i < 4; i++) startTicks[i] = wheels.motors[i].getCurrentPosition();

        telemetry.addLine("<big><b>Wheel Test</b></big>");
        telemetry.addLine("Wheels OFF the ground, then press ▶.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_down && !downWas) selected = (selected + 1) % 4;
            if (gamepad1.dpad_up && !upWas) selected = (selected + 3) % 4;
            upWas = gamepad1.dpad_up;
            downWas = gamepad1.dpad_down;

            if (gamepad1.b && !bWas) {
                wheels.flip(selected);
                message = DriveWheels.LABELS[selected] + " flipped and saved.";
                gamepad1.rumble(200);
            }
            bWas = gamepad1.b;

            double[] p = new double[4];
            if (gamepad1.y) {
                for (int i = 0; i < 4; i++) p[i] = TEST_POWER;
            } else if (gamepad1.a) {
                p[selected] = TEST_POWER;
            }
            wheels.setPowers(p[0], p[1], p[2], p[3]);

            telemetry.addLine("<big><b>Wheel Test</b></big>   <small>(front of robot is up)</small>");
            telemetry.addLine("<tt>" + cell(0, selected, p) + "  " + cell(1, selected, p) + "</tt>");
            telemetry.addLine("<tt>" + cell(2, selected, p) + "  " + cell(3, selected, p) + "</tt>");
            telemetry.addLine("");
            telemetry.addLine(String.format("Selected: <b>%s</b> (port %d, config name %s, %s)",
                    DriveWheels.LABELS[selected], DriveWheels.PORTS[selected], DriveWheels.NAMES[selected],
                    wheels.reversed[selected] ? "reversed" : "not reversed"));
            int ticks = wheels.motors[selected].getCurrentPosition() - startTicks[selected];
            if (Math.abs(ticks) > 50) encoderSeen[selected] = true;
            telemetry.addLine(String.format("Encoder: %d ticks  %s", ticks,
                    encoderSeen[selected] ? (ticks >= 0 ? "<font color='#69f0ae'>connected</font>" : "<font color='#ffb74d'>connected, counts backward</font>")
                                          : "<small>(spin it; stays 0 = no encoder cable, that's OK)</small>"));
            telemetry.addLine("<b>Hold A</b>: only this wheel should spin, rolling FORWARD.");
            telemetry.addLine("Rolls backward? Press <b>B</b> to flip it.");
            telemetry.addLine("A <i>different</i> wheel spins? Its cable is in the wrong port: swap cables.");
            telemetry.addLine("<small>D-pad up/down: pick wheel · Y: all four forward</small>");
            if (!message.isEmpty()) telemetry.addLine("<font color='#69f0ae'>" + message + "</font>");
            telemetry.update();
        }
        wheels.stop();
    }

    private static String cell(int wheel, int selected, double[] p) {
        String name = String.format("%-11s", DriveWheels.LABELS[wheel]);
        String spin = p[wheel] > 0 ? " ▲" : "  ";
        String text = "[" + name + spin + "]";
        return wheel == selected ? "<font color='#ffd54f'><b>" + text + "</b></font>" : text;
    }
}
