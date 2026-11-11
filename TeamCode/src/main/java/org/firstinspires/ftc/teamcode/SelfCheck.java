package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ReadWriteFile;

import java.io.File;

/*
 * Remembers which way the gyro and the two odometry pods count.
 *
 * A gyro or pod that counts the "wrong" way (mounted upside down, cable reversed) would turn a correction
 * into a push in the WRONG direction. So nothing is assumed: Wheel Test step 3 has you push the robot by
 * hand, sees which way each sensor counts, and saves it here (a small file on the Control Hub).
 * Until a sensor has been checked its sign is 0, and whatever would use it stays off.
 *
 * Used by: the autonomous (steering straight) and Wheel Test (measuring the floor test).
 */
public class SelfCheck {

    private static final File SAVE_FILE = new File("/sdcard/FIRST/biobuzz_checks.txt");

    /** +1 = counts the normal way, -1 = counts backward, 0 = not checked yet. */
    public int gyroSign, xPodSign, yPodSign;

    public SelfCheck() {
        load();
    }

    /** Results from Wheel Test's push-by-hand step (0 = not measured, keeps the old value). */
    public void setMeasured(int gyro, int xPod, int yPod) {
        if (gyro != 0) gyroSign = gyro;
        if (xPod != 0) xPodSign = xPod;
        if (yPod != 0) yPodSign = yPod;
        save();
    }

    private void load() {
        if (!SAVE_FILE.exists()) return;
        try {
            for (String line : ReadWriteFile.readFile(SAVE_FILE).split("\n")) {
                String[] parts = line.trim().split("=");
                if (parts.length != 2) continue;
                int v = Integer.parseInt(parts[1].trim());
                if (parts[0].equals("gyro")) gyroSign = v;
                if (parts[0].equals("xPod")) xPodSign = v;
                if (parts[0].equals("yPod")) yPodSign = v;
            }
        } catch (RuntimeException ignored) {
            // A broken file just means everything gets re-checked
        }
    }

    private void save() {
        ReadWriteFile.writeFile(SAVE_FILE, "gyro=" + gyroSign + "\nxPod=" + xPodSign + "\nyPod=" + yPodSign + "\n");
    }
}
