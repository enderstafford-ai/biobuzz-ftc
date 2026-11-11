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
 * Neither working: the autonomous just stops steering itself straight; everything else carries on.
 *
 * Mecanum Drive does not use this at all (plain driving). The autonomous and Wheel Test do.
 *
 * Why the care: asking a sensor that doesn't answer, over and over, jams the Control Hub's
 * motor chip (motors stop, battery reads 0.00 V). So each sensor is checked once at INIT, and
 * dropped for the rest of the run if it stops answering. It never gets asked again.
 */
public class HeadingSensor {

    /** Odometry pod counts per inch of travel (goBILDA 4-bar pod). */
    public static final double POD_TICKS_PER_INCH = 505.3;
    /** Where the pods sit, in inches from the center of the robot (last year's tuned numbers, team 19439, same drivetrain). */
    public static final double X_POD_SIDE_OFFSET = 2.625, Y_POD_BACK_OFFSET = 4.625;

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
                // Pod type and directions from last year's tuned code: X pod right of center, Y pod behind center
                pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
                pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                                              GoBildaPinpointDriver.EncoderDirection.FORWARD);
                pinpoint.setOffsets(-X_POD_SIDE_OFFSET, -Y_POD_BACK_OFFSET, DistanceUnit.INCH);
                pinpoint.resetPosAndIMU();                   // robot must sit still for a moment at INIT
                return;
            }
        } catch (RuntimeException e) {
            pinpoint = null;
            if (pinpointConfigured) problem = "Pinpoint not answering: check its I2C cable goes to I2C port 0 on the Control Hub";
        }
        try {
            imu = hardwareMap.get(IMU.class, "imu");
            // The Control Hub stands on its side: logo facing left, USB ports facing the back (from last year's code)
            boolean ok = imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));
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
        try {
            read();
        } catch (RuntimeException e) {
            // A cable knocked loose can make a read fail outright: count it like a bad read, never crash the drive
            if (++badReads >= GIVE_UP_AFTER) {
                pinpoint = null;
                imu = null;
                problem = "Gyro/Pinpoint stopped answering (loose cable?). Off until restart. Driving still works.";
            }
        }
    }

    private void read() {
        if (pinpoint != null) {
            pinpoint.update();
            if (pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ) {
                if (++badReads >= GIVE_UP_AFTER) {
                    pinpoint = null;
                    problem = "Pinpoint stopped answering (loose cable?). Off until restart";
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
                    problem = "Hub gyro stopped answering. Off until restart";
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

    /** An angle difference folded into -180..180 degrees (so 350 degrees one way reads as 10 the other way). */
    public static double wrap(double degrees) {
        while (degrees > 180) degrees -= 360;
        while (degrees < -180) degrees += 360;
        return degrees;
    }

    /** Raw pod counts, for checking the pod cables (0 when there's no Pinpoint). */
    public int xPodTicks() {
        return pinpoint == null ? 0 : pinpoint.getEncoderX();
    }

    public int yPodTicks() {
        return pinpoint == null ? 0 : pinpoint.getEncoderY();
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
