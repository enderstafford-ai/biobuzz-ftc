package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * BioBuzz 38196 - Simple Drive: last year's drive math (team 19439, DECODE), nothing added.
 * Full power, no speed modes, no gyro, no odometry pods, no smoothing.
 *
 * Use it to compare with Mecanum Drive. If this drives badly too, the problem is in the robot
 * (wheels, belts, motors), not in the code.
 *
 * Left stick: drive and slide. Right stick: spin.
 */
@TeleOp(name = "Simple Drive (last year's math)", group = "BioBuzz")
public class SimpleDrive extends LinearOpMode {

    @Override
    public void runOpMode() {
        DriveWheels wheels = new DriveWheels(hardwareMap);
        telemetry.addLine("Simple Drive: last year's drive math, full power, no gyro. Press ▶.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double forward = -gamepad1.left_stick_y;
            double strafe  =  gamepad1.left_stick_x;
            double turn    =  gamepad1.right_stick_x;

            // Exactly their fieldCentricDrive() with the robot facing straight ahead
            double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1.0);
            double fl = (forward + strafe + turn) / denominator;
            double bl = (forward - strafe + turn) / denominator;
            double fr = (forward - strafe - turn) / denominator;
            double br = (forward + strafe - turn) / denominator;
            wheels.setPowers(fl, fr, bl, br);

            telemetry.addData("Sticks", "fwd %.2f  slide %.2f  spin %.2f", forward, strafe, turn);
            telemetry.addData("Wheels", "FL %.2f  FR %.2f  BL %.2f  BR %.2f", fl, fr, bl, br);
            telemetry.update();
        }
        wheels.stop();
    }
}
