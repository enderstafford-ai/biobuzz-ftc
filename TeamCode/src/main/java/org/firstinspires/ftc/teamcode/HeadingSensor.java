package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/*
 * Which way the robot is facing (and, with the Pinpoint, where it is).
 *
 * Best: the goBILDA Pinpoint Odometry Computer, named "pinpoint" in the robot configuration
 *       (Control Hub I2C Bus 0, type "goBILDA Pinpoint Odometry Computer"). It reads the two
 *       odometry pods (the little spring-loaded omni wheels under the robot) plus its own gyro.
 * Backup: the gyro inside the Control Hub, named "imu". Only used when there's no working Pinpoint.
 * Neither working: heading hold just turns off.
 *
 * Why the care: asking a sensor that doesn't answer, over and over, jams the Control Hub's
 * motor chip (motors stop, battery reads 0.00 V). So each sensor is checked once at INIT, and
 * dropped for the rest of the run if it stops answering. It never gets asked again.
 */
public class HeadingSensor {

    private static final int GIVE_UP_AFTER = 20;   // bad reads in a row before a sensor is dropped

    private GoBildaPinpointDriver pinpoint;
    private IMU imu;
    private double heading;
    private int badReads;
    private String problem = "";

    public HeadingSensor(HardwareMap hardwareMap) {
        boolean pinpointConfigured = false;
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            pinpointConfigured = true;
            // One question first: an unplugged Pinpoint answers all zeros
            if (pinpoint.getDeviceVersion() == 0) {
                pinpoint = null;
                problem = "Pinpoint not answering: check its I2C cable goes to I2C port 0 on the Control Hub";
            } else {
                pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
                pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                                              GoBildaPinpointDriver.EncoderDirection.FORWARD);
                pinpoint.setOffsets(0, 0, DistanceUnit.MM);  // pod positions not measured yet; fine for heading
                pinpoint.resetPosAndIMU();                   // robot must sit still for a moment at INIT
                return;
            }
        } catch (RuntimeException e) {
            pinpoint = null;
            if (pinpointConfigured) problem = "Pinpoint not answering: check its I2C cable goes to I2C port 0 on the Control Hub";
        }
        try {
            imu = hardwareMap.get(IMU.class, "imu");
            // The Control Hub lies flat, logo up
            boolean ok = imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
            if (ok) {
                imu.resetYaw();
            } else {
                imu = null;
                if (!pinpointConfigured) problem = "Hub gyro \"imu\" not answering: wrong type picked in the configuration?";
            }
        } catch (RuntimeException e) {
            imu = null;
        }
    }

    public boolean available() {
        return pinpoint != null || imu != null;
    }

    public boolean hasPosition() {
        return pinpoint != null;
    }

    /** Call once per loop before reading. */
    public void update() {
        if (pinpoint != null) {
            pinpoint.update();
            if (pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ) {
                if (++badReads >= GIVE_UP_AFTER) {
                    pinpoint = null;
                    problem = "Pinpoint stopped answering (loose cable?), heading hold off until restart";
                }
            } else {
                badReads = 0;
                heading = pinpoint.getHeading(AngleUnit.DEGREES);
            }
        } else if (imu != null) {
            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
            if (angles.getAcquisitionTime() == 0) {   // the SDK's "no answer"
                if (++badReads >= GIVE_UP_AFTER) {
                    imu = null;
                    problem = "Hub gyro stopped answering, heading hold off until restart";
                }
            } else {
                badReads = 0;
                heading = angles.getYaw(AngleUnit.DEGREES);
            }
        }
    }

    /** Degrees, counterclockwise is positive. */
    public double heading() {
        return heading;
    }

    public double xInches() {
        return pinpoint == null ? 0 : pinpoint.getPosX(DistanceUnit.INCH);
    }

    public double yInches() {
        return pinpoint == null ? 0 : pinpoint.getPosY(DistanceUnit.INCH);
    }

    /** Raw pod counts, for checking the pod cables (0 when there's no Pinpoint). */
    public int xPodTicks() {
        return pinpoint == null ? 0 : pinpoint.getEncoderX();
    }

    public int yPodTicks() {
        return pinpoint == null ? 0 : pinpoint.getEncoderY();
    }

    public String sourceName() {
        return pinpoint != null ? "Pinpoint" : imu != null ? "hub gyro" : "none";
    }

    /** What's wrong with the gyro/Pinpoint in plain words, or "" when it's fine. */
    public String problem() {
        if (pinpoint == null) return problem;
        switch (pinpoint.getDeviceStatus()) {
            case READY:                     return "";
            case CALIBRATING:               return "Pinpoint starting up, keep the robot still";
            case FAULT_BAD_READ:            return "Pinpoint: a bad read (if it keeps happening, check the I2C cable)";
            case FAULT_NO_PODS_DETECTED:    return "Pinpoint: no pods plugged into X or Y";
            case FAULT_X_POD_NOT_DETECTED:  return "Pinpoint: nothing plugged into X";
            case FAULT_Y_POD_NOT_DETECTED:  return "Pinpoint: nothing plugged into Y";
            case FAULT_IMU_RUNAWAY:         return "Pinpoint gyro error: restart the robot with it sitting still";
            default:                        return "Pinpoint not ready";
        }
    }
}
