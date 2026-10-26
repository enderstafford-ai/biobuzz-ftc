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
 * The intake wheels: one goBILDA 5203 motor, 1620 RPM.
 *
 * Hold LT: intake pulls in.   Hold LB: intake pushes out.   Neither (or both): stopped.
 *
 * Which way is "in" is saved in a small file on the Control Hub. If LT and LB come out backward,
 * press Y on the INIT screen (before ▶) to swap them. No code change or reinstall needed.
 *
 * Robot configuration: any motor that isn't one of the four drive wheels counts as the intake,
 * whatever it was named, so it works with "intake" or any other name.
 */
public class Intake {

    private static final double POWER = 1.0;
    private static final double TRIGGER = 0.3;   // how far LT has to be pulled to count
    private static final File SAVE_FILE = new File("/sdcard/FIRST/biobuzz_intake.txt");

    private final List<DcMotorSimple> motors = new ArrayList<>();
    public final List<String> names = new ArrayList<>();
    private boolean swapped;
    private boolean yWasDown;
    private double power;
    private double amps;
    private int statusCalls;

    public Intake(HardwareMap hardwareMap) {
        List<String> drive = Arrays.asList(DriveWheels.NAMES);
        for (DcMotorSimple m : hardwareMap.getAll(DcMotorSimple.class)) {
            if (m instanceof CRServo) continue;
            String name = hardwareMap.getNamesOf(m).isEmpty() ? "?" : hardwareMap.getNamesOf(m).iterator().next();
            if (drive.contains(name)) continue;
            if (m instanceof DcMotor) {
                ((DcMotor) m).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                ((DcMotor) m).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            motors.add(m);
            names.add(name);
        }
        swapped = SAVE_FILE.exists() && ReadWriteFile.readFile(SAVE_FILE).trim().equals("swapped");
    }

    public boolean available() {
        return !motors.isEmpty();
    }

    /** Call every loop on the INIT screen: Y swaps which button pulls in, and saves it. */
    public boolean checkSwapButton(Gamepad gamepad) {
        boolean pressed = gamepad.y && !yWasDown;
        yWasDown = gamepad.y;
        if (pressed) {
            swapped = !swapped;
            ReadWriteFile.writeFile(SAVE_FILE, swapped ? "swapped" : "normal");
            gamepad.rumble(200);
        }
        return pressed;
    }

    /** Call every loop while driving. */
    public void update(Gamepad gamepad) {
        boolean in = gamepad.left_trigger > TRIGGER;
        boolean out = gamepad.left_bumper;
        double p = in == out ? 0 : in ? POWER : -POWER;
        power = swapped ? -p : p;
        for (DcMotorSimple m : motors) m.setPower(power);
    }

    public void stop() {
        power = 0;
        for (DcMotorSimple m : motors) m.setPower(0);
    }

    /** One line for the screen: what it's doing and how hard the motor is working. */
    public String status() {
        if (!available()) {
            return "<font color='#ffb74d'>Intake: no intake motor in the robot configuration</font>";
        }
        double sign = swapped ? -power : power;
        String doing = sign > 0 ? "<font color='#69f0ae'><b>IN</b></font>"
                     : sign < 0 ? "<font color='#ff8a65'><b>OUT</b></font>" : "off";
        if (statusCalls++ % 10 == 0) {   // current changes slowly; reading it every loop slows the hub down
            amps = 0;
            for (DcMotorSimple m : motors) if (m instanceof DcMotorEx) amps += ((DcMotorEx) m).getCurrent(CurrentUnit.AMPS);
        }
        return String.format("Intake: %s  %.1fA   <small>(LT in · LB out)</small>", doing, amps);
    }

    public String buttonsHelp() {
        if (!available()) return "No intake motor found. Add the intake motor to the robot configuration.";
        return "Intake (" + String.join(", ", names) + "): hold LT = in, LB = out."
             + (swapped ? " [swapped]" : "") + " Backward? Press Y now to swap.";
    }
}
