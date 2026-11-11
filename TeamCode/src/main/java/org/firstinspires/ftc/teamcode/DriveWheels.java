package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ReadWriteFile;

import java.io.File;

/*
 * The four drive motors, in one place, plus which way each one spins.
 *
 * Wheel directions are saved in a small file on the Control Hub, so the Wheel Test
 * op mode can flip a backward wheel without anyone changing code or reinstalling.
 *
 * Robot configuration names (Control Hub motor ports 0-3):
 *   frontLeft (0), backLeft (1), frontRight (2), backRight (3)
 */
public class DriveWheels {

    public static final String[] NAMES  = {"frontLeft", "frontRight", "backLeft", "backRight"};
    public static final String[] LABELS = {"Front left", "Front right", "Back left", "Back right"};
    public static final int[]    PORTS  = {0, 2, 1, 3};

    public static final int FL = 0, FR = 1, BL = 2, BR = 3;

    private static final File SAVE_FILE = new File("/sdcard/FIRST/biobuzz_wheels.txt");

    public final DcMotorEx[] motors = new DcMotorEx[4];
    public final boolean[] reversed = new boolean[4];

    public DriveWheels(HardwareMap hardwareMap) {
        // Starting point: the left side faces the other way, so it's reversed
        reversed[FL] = true;
        reversed[BL] = true;
        load();

        for (int i = 0; i < 4; i++) {
            motors[i] = hardwareMap.get(DcMotorEx.class, NAMES[i]);
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);       // no encoder cables needed
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // stop fast when sticks let go
        }
        applyDirections();
    }

    public void applyDirections() {
        for (int i = 0; i < 4; i++) {
            motors[i].setDirection(reversed[i] ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        }
    }

    /** Flip one wheel's direction and save it so every op mode uses it from now on. */
    public void flip(int wheel) {
        reversed[wheel] = !reversed[wheel];
        applyDirections();
        save();
    }

    /** Free-wheeling (true) lets you push the robot by hand; braking (false) stops it fast when sticks let go. */
    public void setFreeWheel(boolean free) {
        for (DcMotorEx m : motors) {
            m.setZeroPowerBehavior(free ? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void setPowers(double fl, double fr, double bl, double br) {
        motors[FL].setPower(fl);
        motors[FR].setPower(fr);
        motors[BL].setPower(bl);
        motors[BR].setPower(br);
    }

    public void setPowers(double[] p) {
        setPowers(p[FL], p[FR], p[BL], p[BR]);
    }

    public void stop() {
        setPowers(0, 0, 0, 0);
    }

    /**
     * The mecanum math, in one place: how hard each wheel pushes (in FL, FR, BL, BR order) for a drive of
     * forward, strafe (+ = right) and turn (+ = clockwise). If any wheel would need more than full power,
     * all four are scaled down together, so the robot still goes the direction asked, just slower.
     */
    public static double[] mix(double forward, double strafe, double turn) {
        double[] power = {
            forward + strafe + turn,   // front left
            forward - strafe - turn,   // front right
            forward - strafe + turn,   // back left
            forward + strafe - turn,   // back right
        };
        double biggest = 1.0;
        for (double p : power) biggest = Math.max(biggest, Math.abs(p));
        for (int i = 0; i < 4; i++) power[i] /= biggest;
        return power;
    }

    private void load() {
        if (!SAVE_FILE.exists()) return;
        try {
            for (String line : ReadWriteFile.readFile(SAVE_FILE).split("\n")) {
                String[] parts = line.trim().split("=");
                if (parts.length != 2) continue;
                for (int i = 0; i < 4; i++) {
                    if (NAMES[i].equals(parts[0])) reversed[i] = parts[1].equals("REVERSE");
                }
            }
        } catch (RuntimeException ignored) {
            // A broken file just means we keep the starting directions
        }
    }

    private void save() {
        StringBuilder text = new StringBuilder();
        for (int i = 0; i < 4; i++) {
            text.append(NAMES[i]).append('=').append(reversed[i] ? "REVERSE" : "FORWARD").append('\n');
        }
        ReadWriteFile.writeFile(SAVE_FILE, text.toString());
    }
}
