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
 * Left stick: drive and slide. Right stick: spin. LT/LB: intake, X/B: rollers, RT: flywheel (tap twice = stays on), Y/A: flywheel speed, RB: brake
 * (same buttons and INIT-screen settings as Mecanum Drive).
 */
@TeleOp(name = "Simple Drive (last year's math)", group = "BioBuzz")
public class SimpleDrive extends LinearOpMode {

    @Override
    public void runOpMode() {
        DriveWheels wheels = new DriveWheels(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Brake brake = new Brake(hardwareMap);
        Flywheel flywheel = new Flywheel(hardwareMap);
        Rollers rollers = new Rollers(hardwareMap);
        Pad.use(gamepad1);
        telemetry.addLine("Simple Drive: last year's drive math, full power, no gyro. Press ▶.");
        telemetry.addLine(intake.buttonsHelp());
        telemetry.addLine(brake.help());
        telemetry.addLine(flywheel.help());
        telemetry.addLine(rollers.help());
        telemetry.update();
        while (opModeInInit()) {
            Pad.use(gamepad1);
            boolean swapped = intake.checkInitButtons(gamepad1);
            boolean flipped = flywheel.checkFlipButton(gamepad1);
            if (swapped || flipped || brake.checkAdjustButtons(gamepad1)) {
                telemetry.addLine(intake.buttonsHelp());
                telemetry.addLine(brake.help());
                telemetry.addLine(flywheel.help());
                telemetry.addLine(rollers.help());
                telemetry.update();
            }
            sleep(20);
        }

        brake.raiseAfterAdjusting();

        while (opModeIsActive()) {
            Pad.use(gamepad1);
            double forward = -gamepad1.left_stick_y;
            double strafe  =  gamepad1.left_stick_x;
            double turn    =  gamepad1.right_stick_x;

            // Exactly their fieldCentricDrive() with the robot facing straight ahead
            double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1.0);
            double fl = (forward + strafe + turn) / denominator;
            double bl = (forward - strafe + turn) / denominator;
            double fr = (forward - strafe - turn) / denominator;
            double br = (forward + strafe - turn) / denominator;
            if (brake.update(gamepad1)) fl = fr = bl = br = 0;   // brake down: wheels don't fight it
            wheels.setPowers(fl, fr, bl, br);
            intake.update(gamepad1);
            flywheel.update(gamepad1);
            rollers.update(gamepad1);

            telemetry.addData("Sticks", "fwd %.2f  slide %.2f  spin %.2f", forward, strafe, turn);
            telemetry.addLine(intake.buttonsHelp());
            telemetry.addLine(flywheel.help());
            telemetry.addLine(rollers.status());
            telemetry.addData("Wheels", "FL %.2f  FR %.2f  BL %.2f  BR %.2f", fl, fr, bl, br);
            telemetry.update();
        }
        wheels.stop();
        intake.stop();
        brake.raise();
        flywheel.stop();
        rollers.stop();
    }
}
