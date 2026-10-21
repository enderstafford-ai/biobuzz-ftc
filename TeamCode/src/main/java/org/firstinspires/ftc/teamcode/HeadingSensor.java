package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * Which way the robot is facing (and, with the Pinpoint, where it is).
 *
 * Best: the goBILDA Pinpoint Odometry Computer, named "pinpoint" in the robot configuration
 *       (I2C bus it's plugged into, type "goBILDA Pinpoint Odometry Computer"). It reads the two
 *       odometry pods (the little spring-loaded omni wheels under the robot) plus its own gyro.
 * Backup: the gyro inside the Control Hub, named "imu" (new configurations include it).
 * Neither found: heading hold just turns off.
 */
public class HeadingSensor {

    private GoBildaPinpointDriver pinpoint;
    private IMU imu;
    private double heading;

    public HeadingSensor(HardwareMap hardwareMap) {
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
            pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                                          GoBildaPinpointDriver.EncoderDirection.FORWARD);
            pinpoint.setOffsets(0, 0, DistanceUnit.MM);  // pod positions not measured yet; fine for heading
            pinpoint.resetPosAndIMU();                   // robot must sit still for a moment at INIT
            return;
        } catch (RuntimeException e) {
            pinpoint = null;
        }
        try {
            imu = hardwareMap.get(IMU.class, "imu");
            // The Control Hub lies flat, logo up
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
            imu.resetYaw();
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
            heading = pinpoint.getHeading(AngleUnit.DEGREES);
        } else if (imu != null) {
            heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
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

    /** "READY", or what's wrong (for example a pod cable not plugged in). Empty when there's no Pinpoint. */
    public String status() {
        return pinpoint == null ? "" : pinpoint.getDeviceStatus().toString();
    }
}
