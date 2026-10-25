# BioBuzz 38196 robot code

Our code lives in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`:

| File | What it is |
|------|------------|
| `MecanumDrive.java` | TeleOp "Mecanum Drive". Left stick = drive + slide, right stick = spin, D-pad = slow nudge, X = speed (Normal / Fast / Slow). |
| `WheelTest.java` | TeleOp "Wheel Test". Spin one wheel at a time; press B to flip a backward wheel (saved on the robot). |
| `DriveWheels.java` | The four drive motors and their directions, shared by both. |

Robot configuration (Control Hub motor ports): `frontLeft` 0, `backLeft` 1, `frontRight` 2, `backRight` 3, all goBILDA 5202/3/4.

Everything else is the official FTC SDK (v12.0). `upstream` remote = FIRST-Tech-Challenge/FtcRobotController, so SDK updates can be merged in.

## Pinpoint (odometry pods + gyro)
- Pinpoint: X port = forward/back pod, Y port = sideways pod, I2C port → Control Hub **I2C port 0**.
- Configuration: Control Hub → I2C Bus 0 → `goBILDA Pinpoint Odometry Computer`, name `pinpoint`.
- `HeadingSensor` asks the Pinpoint once at INIT and drops it if it doesn't answer. Polling a missing
  I2C device every loop jammed the hub's motor chip (motors dead, battery 0.00 V, "Control Hub not responding").
- Pod settings come from last year's tuned Road Runner code (github.com/Brunana77/19439-DECODE, PinpointDrive.java):
  4-bar pods, X pod REVERSED, Y pod FORWARD, offsets x −2.625", y −4.625". Their Control Hub IMU orientation:
  logo LEFT, USB BACKWARD. A copy sits next to this repo in `last-year-19439-DECODE/`.
- Straight-line hold (Mecanum Drive): on a pure sideways or pure forward push, pod drift off the line is pushed back.
- 10/24: Mecanum Drive starts at FULL power (last year's code had no speed cap; ours was 65%). `SelfCheck` learns
  gyro/pod directions from the driver's own spins and straight pushes (saved in /sdcard/FIRST/biobuzz_checks.txt);
  heading hold and line hold stay off until their sensor is proven. `Simple Drive (last year's math)` = their exact
  drive math, full power, no sensors: the A/B test for "code or hardware?".
- 10/25: Wheel Test is now a 5-step drivetrain check (RB/LB between steps): 1 which wheel is which (box),
  2 roller X-pattern picture, 3 push-by-hand sensor check (saves gyro/pod directions into SelfCheck, detects
  swapped X/Y pod cables), 4 hold-to-move floor test (forward / slide / spin, measured by pods + gyro, motor-amps
  check for slipping belts), 5 results page to photograph.
