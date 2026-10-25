package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ReadWriteFile;

import java.io.File;

/*
 * Learns which way the gyro and the two odometry pods count, by watching the robot while you drive.
 *
 * A gyro or pod that counts the "wrong" way (mounted upside down, cable reversed, settings guessed wrong)
 * turns a correction into a push in the WRONG direction: the robot fights you instead of helping.
 * So nothing is assumed. When you spin with the right stick, the robot sees which way the heading moves;
 * when you drive straight forward or straight sideways, it sees which way each pod counts. Until a sensor
 * has been checked, the correction that uses it stays off. Results are saved on the robot, and keep being
 * re-checked, so a remounted sensor gets re-learned by itself.
 */
public class SelfCheck {

    private static final File SAVE_FILE = new File("/sdcard/FIRST/biobuzz_checks.txt");

    private static final double GYRO_PROOF = 15;    // (spin power x degrees turned) before deciding
    private static final double POD_PROOF = 800;    // (drive power x pod ticks) before deciding, about 3 inches

    // +1 or -1 once learned, 0 = not checked yet
    public int gyroSign, xPodSign, yPodSign;
    private double gyroEvidence, xEvidence, yEvidence;

    public SelfCheck() {
        load();
    }

    /** Driver is spinning with the right stick. Positive turn spins clockwise, so the heading should go down. */
    public void spun(double turn, double headingChangeDeg) {
        gyroEvidence += -turn * headingChangeDeg;
        if (Math.abs(gyroEvidence) > GYRO_PROOF) {
            int learned = gyroEvidence > 0 ? 1 : -1;
            gyroEvidence = 0;
            if (learned != gyroSign) { gyroSign = learned; save(); }
        }
    }

    /** Driver is pushing straight forward/back. Forward pod should count with the push. */
    public void droveForward(double forward, int xTicksChange) {
        xEvidence += forward * xTicksChange;
        if (Math.abs(xEvidence) > POD_PROOF) {
            int learned = xEvidence > 0 ? 1 : -1;
            xEvidence = 0;
            if (learned != xPodSign) { xPodSign = learned; save(); }
        }
    }

    /** Driver is pushing straight sideways (positive = right). Sideways pod should count with the push. */
    public void droveSideways(double strafe, int yTicksChange) {
        yEvidence += strafe * yTicksChange;
        if (Math.abs(yEvidence) > POD_PROOF) {
            int learned = yEvidence > 0 ? 1 : -1;
            yEvidence = 0;
            if (learned != yPodSign) { yPodSign = learned; save(); }
        }
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
